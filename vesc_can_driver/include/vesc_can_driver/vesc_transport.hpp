#pragma once
#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "vesc_can_driver/driver_core.hpp"
namespace vesc_can_driver
{
struct TransportOptions
{
  std::chrono::milliseconds send_period{10};
  std::chrono::milliseconds fault_period{100};  // 0 disables fault queries
  bool auto_activate{false};  // standalone starts on fresh input; hardware uses lifecycle
};

struct DriverFault
{
  std::string reason;
  std::optional<FaultCode> device_fault;  // absent for PC/transport errors
};

/// @brief 専用 executor で VESC の周期送受信を管理
/// @note ros2_control の read/write では ROS 配信を行わない
class VescTransport
{
public:
  /// @brief 通信管理と周期タイマーを作成する，executor は start() で起動
  /// @param configs モータ設定（登録順）
  /// @param opts 送信周期・故障問い合わせの周期
  /// @param io_node 通信専用 executor に登録するノード
  /// @param interface CAN / UART の指令送信と受信値変換の実装
  /// @param on_fault 異常を検出した処理から呼ぶ通知関数，状態ロック外で実行
  VescTransport(std::vector<MotorConfig> configs, TransportOptions opts,
    rclcpp::Node::SharedPtr io_node, std::unique_ptr<VescInterface> interface,
    std::function<void(const DriverFault &)> on_fault = {});

  /// @brief shutdown() を呼び，専用スレッドと通信処理を終了
  ~VescTransport();

  /// @brief 通信リソースのコピーを禁止する
  VescTransport(const VescTransport &) = delete;

  /// @brief 通信リソースのコピー代入を禁止する
  VescTransport &operator=(const VescTransport &) = delete;

  /// @brief 専用スレッドで通信 executor を起動
  /// @throws std::logic_error 起動済み，または終了済み
  void start();

  /// @brief executor を停止・join し，ROS が有効なら最後の停止送信を試みる
  /// @note 複数回呼び出し可
  void shutdown() noexcept;

  /// @brief 全台の必要なフィードバックが揃うまで待機
  /// @return 準備完了で true，時間切れ・終了要求で false
  bool wait_ready(std::chrono::milliseconds timeout);

  /// @brief フィードバックを確認して有効化し，成功時に旧指令と異常を解除
  /// @return 有効化に成功した場合は true
  bool activate();

  /// @brief 動作指令を無効化し，以後の送信をゼロ電流に切り替える
  void deactivate();

  /// @brief 指定モータの再起動を 通信 executor へ要求する
  /// @return 要求受理で true，添字不正・終了済み・要求待ち・再起動待機中は false
  /// @note hardware の動作中は受理しない，旧指令は全台分を破棄
  bool reboot(std::size_t motor_idx);

  /// @brief 通常のコールバックから理由付きで異常停止し，初回の異常を即時通知
  /// @note ロックを取得するため ros2_control の read/write では request_stop() を使用
  void stop_on_error(const std::string &reason);

  /// @brief ロックを待たずに全台の異常停止を要求する
  void request_stop() noexcept
  {
    fault_requested_.store(true);
    stop_requested_.store(true);
  }

  /// @brief 通信側で保持する動作許可状態を返す
  bool active() const
  {
    return active_.load();
  }

  /// @brief 通信側で保持する異常停止状態を返す
  bool faulted() const
  {
    return faulted_.load();
  }

  /// @brief ロックを待たずに全台の最新指令を保存
  /// @param values 登録順の指令値配列，単位は各モータのモードに従う
  /// @return 保存時は true，不正入力・無効状態・ロック競合で false
  bool try_set_commands(const std::vector<double> &values);

  /// @brief 指定モータの最新指令を配列ごとに一括保存する
  /// @param cmds モータ添字と固定モードの指令値，未指定モータは更新しない
  /// @return 受理時は true，不正配列・停止保持・再起動待機中は false
  /// @note 同じモータの未送信値を上書きし，全要素を 1 回のロック・同じ受信時刻で保存
  bool set_commands(const std::vector<MotorCommand> &cmds);

  /// @brief ロックを待たずに状態を事前確保済みの配列へコピー
  /// @param[out] states モータ数と同じサイズの出力配列
  /// @return コピー時は true，サイズ不一致・ロック競合で false
  bool try_snapshot(std::vector<MotorState> &states);

  /// @brief ロックを取得し，全台の最終受信状態を登録順でコピー
  std::vector<MotorState> snapshot();

  /// @brief ロックを取得し，保持している異常停止理由を返す
  std::string fault_reason();

private:
  struct PendingCommand
  {
    double value{};
    std::optional<TimePoint> received_at;
  };

  /// @brief 周期タイマーで最新指令を取り出し，検査・生成して配信
  void send_commands();

  /// @brief 停止要求を反映し，共通指令を通信実装へ渡す
  void send_outputs();

  /// @brief 登録モータへ故障コードを問い合わせる，停止中も継続
  void poll_faults();

  /// @brief 通信実装が確認した受信値を反映し，異常を即時通知
  void receive_feedback(const VescFeedback &feedback);

  /// @brief 状態ロック外で異常通知を呼び，通知側の例外を記録する
  void notify_fault(const DriverFault &fault);

  DriverCore core_;
  TransportOptions opts_;
  rclcpp::Node::SharedPtr io_node_;
  std::function<void(const DriverFault &)> on_fault_;
  std::unique_ptr<VescInterface> interface_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::TimerBase::SharedPtr fault_timer_;
  // Reboot guards are protected by state_mutex_.
  std::vector<TimePoint> rx_after_;
  rclcpp::executors::SingleThreadedExecutor io_exec_;
  std::thread exec_thread_;
  std::mutex state_mutex_;
  std::condition_variable feedback_cv_;
  std::vector<PendingCommand> pending_cmds_;
  std::optional<std::size_t> reboot_idx_;
  std::vector<VescCommand> outputs_;
  std::vector<VescCommand> fault_commands_;
  std::atomic<bool> shutdown_started_{false};
  std::atomic<bool> stop_requested_{true};
  std::atomic<bool> active_{false};
  std::atomic<bool> faulted_{false};
  std::atomic<bool> fault_requested_{false};
};
}  // namespace vesc_can_driver
