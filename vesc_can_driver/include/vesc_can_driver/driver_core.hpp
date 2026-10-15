#pragma once
#include <vector>

#include "vesc_can_driver/motor_controller.hpp"
namespace vesc_can_driver
{
/// @brief モータ一覧の設定範囲と，名前・論理 ID・接続先 ID の重複を検査
void validate_configs(const std::vector<MotorConfig> &configs);

/// @brief 複数モータの受信先振り分け・全台の動作許可・異常停止を管理
/// @note VescTransport ごとに 1 個所有し，全台操作の対象はこのインスタンスの登録モータのみ
/// @note 呼び出し側でアクセスを直列化，受信は無効・異常停止中も継続
class DriverCore
{
public:
  /// @brief モータ設定を検査し，状態と指令の保存領域を初期化
  explicit DriverCore(std::vector<MotorConfig> configs);

  /// @brief 登録順のモータを参照，設定と受信状態は各モータが所有
  const std::vector<MotorController> &motors() const
  {
    return motors_;
  }

  /// @brief 登録 ID のモータへデコード済み受信値を振り分ける
  /// @param feedback デコード済みの受信値
  /// @param now 受信した単調時刻
  /// @return 状態を更新した場合は true
  bool receive(const VescFeedback &feedback, TimePoint now);

  /// @brief 登録 ID の故障コードを更新し，非ゼロなら全台を異常停止
  /// @note コード 0 の受信で PC 側の異常停止は解除しない
  bool receive_fault(const FaultCode &fault, TimePoint now);

  /// @brief 全台に既知の本体故障がなく，フィードバックと位置条件を満たすか判定
  bool ready(TimePoint now) const;

  /// @brief 全台の準備を確認し，PC 側で動作指令を許可
  /// @return 有効化に成功した場合は true
  /// @note 成功時に旧指令と異常を解除，CAN 有効化パケットは送らない
  bool activate(TimePoint now);

  /// @brief 動作指令を無効にし，保存した指令をリセット
  void deactivate();

  /// @brief 再起動する 1 台の状態と，全台の旧指令・PC 側の異常をリセット
  bool reset(std::size_t motor_idx);

  /// @brief 全台を無効化し，異常停止の理由を保持
  void stop_on_error(const std::string &reason);

  /// @brief 動作指令が許可されているか
  bool is_active() const
  {
    return active_;
  }

  /// @brief 復旧操作が必要な異常停止状態か
  bool is_faulted() const
  {
    return faulted_;
  }

  /// @brief 保持している異常停止理由を参照
  const std::string &fault_reason() const
  {
    return fault_reason_;
  }

  /// @brief モードの単位で指令値と受信時刻を更新
  /// @param idx 登録順のモータ添字
  /// @param value 電流 [A]，関節速度 [rad/s]，関節位置 [rad] または duty 比
  /// @param now 指令を受信した単調時刻
  /// @return 受理時は true，不正値では全台を異常停止
  bool set_command(std::size_t idx, double value, TimePoint now);

  /// @brief 最新指令から全台分の共通指令を生成し，無効・異常時はゼロ電流に
  /// @param now 期限判定と速度ランプに使う単調時刻
  /// @param[out] frames 生成結果で上書きする共通指令配列
  void commands(TimePoint now, std::vector<VescCommand> &commands);

  /// @brief 全台分のゼロ電流共通指令で配列を上書き
  void stop_commands(std::vector<VescCommand> &commands) const;

private:
  std::vector<MotorController> motors_;
  bool active_{false};
  bool faulted_{false};
  std::string fault_reason_;
};

}  // namespace vesc_can_driver
