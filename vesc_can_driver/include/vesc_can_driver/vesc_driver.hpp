#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <mutex>
#include <numbers>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "vesc_can_driver/vesc_protocol.hpp"

namespace vesc_can_driver
{
using SteadyClock = std::chrono::steady_clock;
using TimePoint = SteadyClock::time_point;
constexpr double kPi = std::numbers::pi_v<double>;
constexpr double kUnknownValue = std::numeric_limits<double>::quiet_NaN();
enum class ControlMode
{
  Current,
  Velocity,
  Position,
  CurrentVelocity,
  DutyCycle
};

/// @brief モード名を ControlMode に変換する
/// @throws std::invalid_argument 未対応のモード名
ControlMode parse_control_mode(std::string_view mode_name);
/// @brief モードに対応する current / velocity / position / duty_cycle を返す
std::string_view command_interface(ControlMode mode);
/// @brief モードを選択し，command interface との整合性を確認する
/// @note モード名が空なら interface 名から選択する
ControlMode resolve_control_mode(std::string_view mode_name, std::string_view interface_name);
/// @brief 速度指令を使う velocity / current_velocity か判定する
bool is_velocity_mode(ControlMode mode);

struct MotorConfig
{
  std::string name;
  std::uint8_t motor_id{};  // ROS logical ID, independent of CAN addressing
  std::uint8_t controller_id{};
  ControlMode mode{ControlMode::Velocity};
  double pole_pairs{7.0};
  double gear_ratio{1.0};  // motor revolutions / joint revolution
  double direction{1.0};
  double zero_offset_rad{0.0};  // motor PID angle at joint position zero
  double max_vel_rad_s{20.0};   // joint rad/s
  double max_current_a{10.0};   // motor A, signed in the joint direction
  double max_duty{1.0};         // absolute duty ratio, in (0, 1]
  double min_pos_rad{-kPi};
  double max_pos_rad{kPi};
  double vel_slew_rad_s2{100.0};  // joint rad/s^2
  double startup_current_a{0.0};  // required for CurrentVelocity
  double rpm_threshold{1000.0};   // motor mechanical RPM
  std::chrono::milliseconds startup_timeout{1000};
  std::chrono::milliseconds feedback_timeout{500};
  std::chrono::milliseconds command_timeout{500};
  bool require_tachometer{false};
};

struct MotorState
{
  // Joint values include gear ratio/direction; native motor values retain VESC units/sign.
  double pos_rad{kUnknownValue};  // STATUS4 in position mode, STATUS5 otherwise
  double vel_rad_s{kUnknownValue};
  double motor_current_a{kUnknownValue};
  double temperature_fet_c{kUnknownValue};
  double temperature_motor_c{kUnknownValue};
  double input_voltage_v{kUnknownValue};
  double duty_cycle{kUnknownValue};
  double electrical_rpm{kUnknownValue};
  double input_current_a{kUnknownValue};
  double pid_pos_deg{kUnknownValue};
  double charge_drawn_ah{kUnknownValue};
  double charge_regen_ah{kUnknownValue};
  double energy_drawn_wh{kUnknownValue};
  double energy_regen_wh{kUnknownValue};
  std::int32_t tachometer_counts{};
  double adc_1_v{kUnknownValue};
  double adc_2_v{kUnknownValue};
  double adc_3_v{kUnknownValue};
  double ppm{kUnknownValue};    // normalized PPM input, dimensionless
  std::int32_t fault_code{-1};  // -1: unknown, 0: confirmed healthy, >0: VESC fault
  std::optional<TimePoint> fault_at;
  // nullopt means not received; a received timestamp may legitimately be TimePoint{}.
  std::optional<TimePoint> status1_at;
  std::optional<TimePoint> status2_at;
  std::optional<TimePoint> status3_at;
  std::optional<TimePoint> status4_at;
  std::optional<TimePoint> status5_at;
  std::optional<TimePoint> status6_at;
};

struct MotorCommand
{
  std::size_t motor_idx{};
  double value{};  // Units follow the motor's fixed control mode.
};

/// @brief 関節速度 [rad/s] を VESC の電気回転数 [ERPM] に変換
double vel_to_erpm(const MotorConfig &cfg, double vel_rad_s);

/// @brief 電気回転数 [ERPM] を減速比と方向込みの関節速度 [rad/s] に変換
double erpm_to_vel(const MotorConfig &cfg, double electrical_rpm);

/// @brief STATUS5 の符号付きカウントを積算関節角度 [rad] に変換
/// @note 極対数・減速比・方向を適用し，2π で折り返さない
double tacho_to_pos(const MotorConfig &cfg, std::int32_t counts);

/// @brief 関節位置 [rad] をオフセット込みのモータ PID 角度 [deg] に変換
/// @return [0, 360) の角度 [deg]
double pos_to_deg(const MotorConfig &cfg, double pos_rad);

/// @brief モータ PID 角度 [deg] を単回転の関節位置 [rad] に変換
double deg_to_pos(const MotorConfig &cfg, double pid_pos_deg);

/// @brief direction（±1）で電流 [A] や duty 比のモータ方向と関節方向を相互変換
double apply_direction(const MotorConfig &cfg, double value);

/// @brief モータ設定の範囲・時間・名前と ID の重複を検査
/// @throws std::invalid_argument 空の一覧または不正な設定
void validate_configs(const std::vector<MotorConfig> &configs);

/// @brief 受信済みで，時刻が未来でも期限切れでもないか判定
/// @note 期限までは有効，未受信は無効
bool feedback_fresh(
  const std::optional<TimePoint> &received_at, TimePoint now, std::chrono::milliseconds timeout);

/// @brief 指令値の有限性と，選択したモードの範囲を検査
bool command_is_valid(const MotorConfig &cfg, double value);

/// @brief 既知の本体故障・必要な STATUS の鮮度・位置範囲から準備状態を判定
bool state_ready(const MotorConfig &cfg, const MotorState &state, TimePoint now);

/// @brief 状態更新と 5 モードの制御判断
/// @note 呼び出し側でアクセスを直列化，受信は無効・異常停止中も継続
class DriverCore
{
public:
  /// @brief モータ設定を検査し，状態と指令の保存領域を初期化
  explicit DriverCore(std::vector<MotorConfig> configs);

  /// @brief 登録順のモータ設定を参照
  const std::vector<MotorConfig> &motor_configs() const
  {
    return configs_;
  }

  /// @brief 登録順の最終受信状態を参照，データの信頼性は受信時刻で判定
  const std::vector<MotorState> &states() const
  {
    return states_;
  }

  /// @brief 登録 ID の STATUS1〜6 をデコードして状態を更新
  /// @param frame 受信 CAN フレーム
  /// @param now 受信した単調時刻
  /// @return 状態を更新した場合は true
  bool receive(const can_msgs::msg::Frame &frame, TimePoint now);

  /// @brief 登録 ID の故障コードを更新し，非ゼロなら全台を異常停止
  /// @note コード 0 の受信で PC 側の異常停止は解除しない
  bool receive_fault(const protocol::FaultCode &fault, TimePoint now);

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
  bool active() const
  {
    return active_;
  }

  /// @brief 復旧操作が必要な異常停止状態か
  bool faulted() const
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

  /// @brief 最新指令から全台分のフレームを生成し，無効・異常時はゼロ電流に
  /// @param now 期限判定と速度ランプに使う単調時刻
  /// @param[out] frames 生成結果で上書きするフレーム配列
  void command_frames(TimePoint now, std::vector<can_msgs::msg::Frame> &frames);

  /// @brief 全台分のゼロ電流フレームで配列を上書き
  void stop_frames(std::vector<can_msgs::msg::Frame> &frames) const;

private:
  struct Command
  {
    double target{};  // joint-direction A/duty ratio, joint rad/s or joint rad by mode
    double ramp_vel{};
    std::optional<TimePoint> received_at;
    TimePoint ramp_at{};
    TimePoint startup_at{};
    bool rpm_control_active{false};
  };

  /// @brief 全台のフィードバックと指令期限を検査し，異常時は停止
  bool check_commands(TimePoint now);

  /// @brief 5 モードの指令生成と電流起動から速度制御への移行
  /// @return 生成フレーム，異常時は nullopt
  std::optional<can_msgs::msg::Frame> make_frame(std::size_t idx, TimePoint now);
  std::vector<MotorConfig> configs_;
  std::vector<MotorState> states_;
  std::vector<Command> cmds_;
  bool active_{false};
  bool faulted_{false};
  TimePoint active_at_{};
  std::string fault_reason_;
};

struct TransportOptions
{
  std::string tx_topic{"can/tx"};
  std::string rx_topic{"can/rx"};
  std::chrono::milliseconds send_period{10};
  std::chrono::milliseconds fault_period{100};  // 0 disables fault queries
  bool auto_activate{false};  // standalone starts on fresh input; hardware uses lifecycle
};

struct DriverFault
{
  std::string reason;
  std::optional<protocol::FaultCode> device_fault;  // absent for PC/transport errors
};

/// @brief 専用 executor で CAN の周期送受信を管理
/// @note ros2_control の read/write では ROS 配信を行わない
class CanTransport
{
public:
  /// @brief CAN トピックと周期タイマーを作成する，executor は start() で起動
  /// @param configs モータ設定（登録順）
  /// @param opts トピック名・送信周期・故障問い合わせの周期
  /// @param can_node 通信専用 executor に登録するノード
  /// @param on_fault 異常を検出した処理から呼ぶ通知関数，状態ロック外で実行
  CanTransport(std::vector<MotorConfig> configs, TransportOptions opts,
    rclcpp::Node::SharedPtr can_node, std::function<void(const DriverFault &)> on_fault = {});

  /// @brief shutdown() を呼び，専用スレッドと通信処理を終了
  ~CanTransport();

  /// @brief 通信リソースのコピーを禁止する
  CanTransport(const CanTransport &) = delete;

  /// @brief 通信リソースのコピー代入を禁止する
  CanTransport &operator=(const CanTransport &) = delete;

  /// @brief 専用スレッドで CAN executor を起動
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

  /// @brief 指定モータの再起動を CAN executor へ要求する
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

  /// @brief 停止要求を反映し，ROS 時刻を付けて CAN フレームを配信
  void publish_frames();

  /// @brief 登録モータへ故障コードを問い合わせる，停止中も継続
  void poll_faults();

  /// @brief STATUS または要求済みの故障応答を処理し，異常を即時通知
  void receive_frame(const can_msgs::msg::Frame &frame);

  /// @brief 状態ロック外で異常通知を呼び，通知側の例外を記録する
  void notify_fault(const DriverFault &fault);

  DriverCore core_;
  TransportOptions opts_;
  rclcpp::Node::SharedPtr can_node_;
  std::function<void(const DriverFault &)> on_fault_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr tx_pub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr rx_sub_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::TimerBase::SharedPtr fault_timer_;
  // Only the dedicated CAN executor accesses pending query times.
  std::vector<std::optional<TimePoint>> fault_requests_;
  std::vector<TimePoint> rx_after_;
  rclcpp::executors::SingleThreadedExecutor can_exec_;
  std::thread exec_thread_;
  std::mutex state_mutex_;
  std::condition_variable feedback_cv_;
  std::vector<PendingCommand> pending_cmds_;
  std::optional<std::size_t> reboot_idx_;
  std::vector<can_msgs::msg::Frame> cmd_frames_;
  std::atomic<bool> shutdown_started_{false};
  std::atomic<bool> stop_requested_{true};
  std::atomic<bool> active_{false};
  std::atomic<bool> faulted_{false};
  std::atomic<bool> fault_requested_{false};
};
}  // namespace vesc_can_driver
