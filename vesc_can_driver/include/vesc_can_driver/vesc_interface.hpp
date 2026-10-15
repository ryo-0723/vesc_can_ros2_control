#pragma once

#include <cstdint>
#include <functional>
#include <span>
#include <string>
#include <variant>

namespace vesc_can_driver
{
/// @brief 通信方式によらない VESC の操作
/// @note value は VESC 方向の A・ERPM・deg・duty 比，問い合わせと再起動では未使用
enum class VescCommandType
{
  Duty,
  Current,
  Rpm,
  Position,
  FaultRequest,
  Reboot
};
struct VescCommand
{
  std::uint8_t controller_id{};
  VescCommandType type{VescCommandType::Current};
  double value{};
};

struct FaultCode
{
  std::uint8_t controller_id{};
  std::uint8_t code{};
};

struct MotionFeedback
{
  std::uint8_t controller_id{};
  std::int32_t electrical_rpm{};
  double motor_current_a{};
  double duty_cycle{};
};
struct ChargeFeedback
{
  std::uint8_t controller_id{};
  double charge_drawn_ah{};
  double charge_regen_ah{};
};
struct EnergyFeedback
{
  std::uint8_t controller_id{};
  double energy_drawn_wh{};
  double energy_regen_wh{};
};
struct ControlFeedback
{
  std::uint8_t controller_id{};
  double temperature_fet_c{};
  double temperature_motor_c{};
  double input_current_a{};
  double motor_pid_position_deg{};
};
struct PositionFeedback
{
  std::uint8_t controller_id{};
  std::int32_t tachometer_counts{};
  double input_voltage_v{};
};
struct AnalogFeedback
{
  std::uint8_t controller_id{};
  double adc_1_v{};
  double adc_2_v{};
  double adc_3_v{};
  double ppm{};  // normalized PPM input, dimensionless
};

/// @brief デコード済みの受信値，UART の応答も取得できた項目のブロックに分けて渡す
using VescFeedback = std::variant<MotionFeedback, ChargeFeedback, EnergyFeedback, ControlFeedback,
  PositionFeedback, AnalogFeedback, FaultCode>;

/// @brief CAN / UART などの通信方式を差し替える境界
/// @note send は通信スレッドから呼び，read/write からは呼ばない
/// @note feedback は完成したブロックだけ通知し，欠測項目をゼロで補わない
class VescInterface
{
public:
  using FeedbackCallback = std::function<void(const VescFeedback &)>;
  using ErrorCallback = std::function<void(const std::string &)>;
  virtual ~VescInterface() = default;

  /// @brief 接続先 ID を登録し，この通信方式で使用できるか検査する
  virtual void configure_targets(std::span<const std::uint8_t> ids) = 0;
  /// @brief 受信と異常の通知先を設定し，必要な受信処理を開始する
  /// @note 開始後は別スレッドから通知してもよい
  virtual void start(FeedbackCallback on_feedback, ErrorCallback on_error) = 0;
  /// @brief 通信処理を終了し，戻った後は通知先を呼ばない
  /// @note 受信スレッドを持つ実装はここで join する，複数回呼び出し可
  virtual void stop() noexcept = 0;
  /// @brief 1 台の共通指令を通信方式の形式へ変換して送る
  /// @note 同じ呼び出し元の送信順序を保ち，正常送信時にブロックし続けない
  virtual void send(const VescCommand &command) = 0;
};
}  // namespace vesc_can_driver
