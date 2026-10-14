#pragma once

#include <cstdint>

#include "can_msgs/msg/frame.hpp"

namespace vesc_can_driver::protocol
{
constexpr std::uint8_t kBroadcastControllerId = 255;
constexpr std::uint32_t kControllerIdMask = 0xFF;
constexpr std::uint32_t kPacketIdShift = 8;
constexpr std::uint32_t kExtendedCanIdMask = 0x1FFFFFFF;

constexpr std::uint32_t kSetCurrentId = 1;
constexpr std::uint32_t kSetRpmId = 3;
constexpr std::uint32_t kSetPositionId = 4;
constexpr std::uint32_t kProcessShortBufferId = 8;
constexpr std::uint8_t kGetValuesSelectiveId = 50;
constexpr std::uint8_t kCommRebootId = 29;
// Native CAN STATUS1-6 layout verified against VESC firmware release_6_06,
// commit 94b305ec124b67b0161f2d3d85c4d5b35e48edc5 (comm/comm_can.c, datatypes.h).
constexpr std::uint32_t kStatus1Id = 9;
constexpr std::uint32_t kStatus2Id = 14;
constexpr std::uint32_t kStatus3Id = 15;
constexpr std::uint32_t kStatus4Id = 16;
constexpr std::uint32_t kStatus5Id = 27;
constexpr std::uint32_t kStatus6Id = 58;

struct FaultCode
{
  std::uint8_t controller_id{};
  std::uint8_t code{};
};

/// @brief CAN の short buffer で VESC 故障コードだけを問い合わせる
/// @param host_id 応答先となる PC の CAN ID（他の機器と重複不可）
/// @throws std::invalid_argument ID が 255 または送受信 ID が同一
can_msgs::msg::Frame make_fault_request(std::uint8_t controller_id, std::uint8_t host_id);

/// @brief 指定 VESC の COMM_REBOOT を short buffer で送るフレームを生成
/// @param host_id PC の CAN ID
/// @return 拡張フレーム（DLC=3），応答は要求しない
/// @throws std::invalid_argument ID が 255 または送受信 ID が同一
can_msgs::msg::Frame make_reboot_frame(std::uint8_t controller_id, std::uint8_t host_id);

/// @brief 故障コードのみを含む selective 応答を検査して読み取る
/// @note CAN ID の下位 byte は宛先，送信元 VESC ID は data[0]
/// @return 正しい形式なら true，不正フレームでは出力を変更しない
bool decode_fault_reply(const can_msgs::msg::Frame &frame, std::uint8_t host_id, FaultCode &fault);

struct Status1
{
  std::uint8_t controller_id{};
  std::int32_t electrical_rpm{};
  double motor_current_a{};
  double duty_cycle{};
};
struct Status2
{
  std::uint8_t controller_id{};
  double charge_drawn_ah{};
  double charge_regen_ah{};
};
struct Status3
{
  std::uint8_t controller_id{};
  double energy_drawn_wh{};
  double energy_regen_wh{};
};
struct Status4
{
  std::uint8_t controller_id{};
  double temperature_fet_c{};
  double temperature_motor_c{};
  double input_current_a{};
  double motor_pid_position_deg{};
};
struct Status5
{
  std::uint8_t controller_id{};
  std::int32_t tachometer_counts{};
  double input_voltage_v{};
};
struct Status6
{
  std::uint8_t controller_id{};
  double adc_1_v{};
  double adc_2_v{};
  double adc_3_v{};
  double ppm{};  // normalized PPM input, dimensionless
};

/// @brief ERPM の速度指令フレームを生成
/// @param controller_id 宛先 ID（0〜254）
/// @param electrical_rpm 電気回転数 [ERPM]
/// @return 拡張 CAN フレーム（DLC=4）
can_msgs::msg::Frame make_set_rpm_frame(std::uint8_t controller_id, std::int32_t electrical_rpm);

/// @brief モータ電流の指令フレームを生成
/// @param controller_id 宛先 ID（0〜254）
/// @param motor_current_a VESC 方向のモータ電流 [A]
/// @return 拡張 CAN フレーム（DLC=4）
/// @throws std::out_of_range 非有限値または倍率変換後の int32 範囲外
can_msgs::msg::Frame make_set_current_frame(std::uint8_t controller_id, double motor_current_a);

/// @brief モータ PID 角度の指令フレームを生成
/// @param controller_id 宛先 ID（0〜254）
/// @param motor_pid_position_deg モータ PID 角度 [deg]
/// @return 拡張 CAN フレーム（DLC=4）
/// @throws std::out_of_range 非有限値または倍率変換後の int32 範囲外
can_msgs::msg::Frame make_set_position_frame(
  std::uint8_t controller_id, double motor_pid_position_deg);

/// @brief STATUS1 の ERPM・モータ電流・duty を読み取る
/// @param frame 受信 CAN フレーム
/// @param[out] status 正常時に更新するデコード結果
/// @return 正しい形式なら true，不正フレームでは出力を変更しない
bool decode_status_1(const can_msgs::msg::Frame &frame, Status1 &status);

/// @brief STATUS2 の積算電荷量を読み取る
/// @param frame 受信 CAN フレーム
/// @param[out] status 正常時に更新するデコード結果
/// @return 正しい形式なら true，不正フレームでは出力を変更しない
bool decode_status_2(const can_msgs::msg::Frame &frame, Status2 &status);

/// @brief STATUS3 の積算電力量を読み取る
/// @param frame 受信 CAN フレーム
/// @param[out] status 正常時に更新するデコード結果
/// @return 正しい形式なら true，不正フレームでは出力を変更しない
bool decode_status_3(const can_msgs::msg::Frame &frame, Status3 &status);

/// @brief STATUS4 の温度・入力電流・PID 角度を読み取る
/// @param frame 受信 CAN フレーム
/// @param[out] status 正常時に更新するデコード結果
/// @return 正しい形式なら true，不正フレームでは出力を変更しない
bool decode_status_4(const can_msgs::msg::Frame &frame, Status4 &status);

/// @brief STATUS5 のタコメータ・入力電圧を読み取る
/// @param frame 受信 CAN フレーム
/// @param[out] status 正常時に更新するデコード結果
/// @return 正しい形式なら true，不正フレームでは出力を変更しない
bool decode_status_5(const can_msgs::msg::Frame &frame, Status5 &status);

/// @brief STATUS6 の ADC 電圧・PPM 入力を読み取る
/// @param frame 受信 CAN フレーム
/// @param[out] status 正常時に更新するデコード結果
/// @return 正しい形式なら true，不正フレームでは出力を変更しない
bool decode_status_6(const can_msgs::msg::Frame &frame, Status6 &status);
}  // namespace vesc_can_driver::protocol
