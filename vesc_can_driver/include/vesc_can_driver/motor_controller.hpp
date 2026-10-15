#pragma once
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <string>
#include <string_view>

#include "vesc_can_driver/can_protocol.hpp"
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

/// @brief モードに対応する command interface 名を返す
std::string_view command_interface(ControlMode mode);

/// @brief モードを選択し，command interface との整合性を確認する
/// @note モード名が空なら interface 名から選択する
ControlMode resolve_control_mode(std::string_view mode_name, std::string_view interface_name);

/// @brief 速度指令を使う velocity / current_velocity か判定する
bool is_velocity_mode(ControlMode mode);

struct MotorConfig
{
  std::string name;
  std::uint8_t logical_id{};  // ROS logical ID, independent of device addressing
  std::uint8_t motor_id{};
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

/// @brief 受信済みで，時刻が未来でも期限切れでもないか判定
/// @note 期限までは有効，未受信は無効
bool feedback_fresh(
  const std::optional<TimePoint> &received_at, TimePoint now, std::chrono::milliseconds timeout);

/// @brief 指令値の有限性と，選択したモードの範囲を検査
bool command_is_valid(const MotorConfig &cfg, double value);

/// @brief 既知の本体故障・必要な STATUS の鮮度・位置範囲から準備状態を判定
bool state_ready(const MotorConfig &cfg, const MotorState &state, TimePoint now);

/// @brief 1 台分の設定・受信状態・指令と 5 モードの制御を管理
/// @note 呼び出し側でアクセスを直列化し，全台の動作許可と停止は DriverCore が管理
class MotorController
{
public:
  struct CommandResult
  {
    can_msgs::msg::Frame command;
    std::string_view error;  // Empty on success; otherwise a static error description.
  };

  /// @brief 1 台分の設定を検査し，状態と指令を初期化
  explicit MotorController(MotorConfig config);
  /// @brief 1 台分の設定範囲を検査する
  static void validate_config(const MotorConfig &config);

  /// @brief このモータの固定設定を参照
  const MotorConfig &config() const
  {
    return config_;
  }
  /// @brief 最終受信状態を参照，鮮度は各 STATUS の受信時刻で判定
  const MotorState &state() const
  {
    return state_;
  }

  /// @brief この接続先のデコード済み受信値を反映し，関節単位へ変換
  bool receive(const VescFeedback &feedback, TimePoint now);
  /// @brief この接続先の故障コードと受信時刻を更新
  bool receive_fault(const FaultCode &fault, TimePoint now);
  /// @brief 受信状態と指令を初期化
  void reset();
  /// @brief 保存指令・停止保持・速度ランプ・電流起動を初期化
  /// @param now 初回指令までの期限の起点
  void reset_command(TimePoint now = TimePoint{});
  /// @brief モードの範囲を確認し，最新指令と受信時刻を保存
  bool set_command(double value, TimePoint now);
  /// @brief 必須フィードバックと指令期限を確認し，正常なら空文字列を返す
  std::string_view command_error(TimePoint now) const;
  /// @brief このモータのCAN 指令フレームを生成し，失敗時は error に理由を返す
  /// @note 異常時は単体の停止を保持し，error とゼロ電流指令を返す
  CommandResult command(TimePoint now);
  /// @brief このモータのゼロ電流フレームを生成
  can_msgs::msg::Frame stop_command() const;

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

  MotorConfig config_;
  MotorState state_;
  Command cmd_;
  TimePoint command_started_at_{};
  std::string_view fault_reason_;
};

}  // namespace vesc_can_driver
