#pragma once

#include <cstdint>
#include <optional>

#include "can_msgs/msg/frame.hpp"
#include "vesc_can_driver/vesc_interface.hpp"

namespace vesc_can_driver::protocol
{
constexpr std::uint8_t kBroadcastControllerId = 255;
// Selective 故障応答の宛先に予約する固定 ID（モータには割り当てない）
constexpr std::uint8_t kReplyCanId = 254;
constexpr std::uint32_t kControllerIdMask = 0xFF;
constexpr std::uint32_t kPacketIdShift = 8;
constexpr std::uint32_t kExtendedCanIdMask = 0x1FFFFFFF;

constexpr std::uint32_t kSetDutyId = 0;
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

using FaultCode = vesc_can_driver::FaultCode;

/// @brief CAN の short buffer で VESC 故障コードだけを問い合わせる
/// @note 応答先は kReplyCanId に固定
/// @throws std::invalid_argument ID が予約済みの 254 または broadcast の 255
can_msgs::msg::Frame make_fault_request(std::uint8_t controller_id);

/// @brief 指定 VESC の COMM_REBOOT を short buffer で送るフレームを生成
/// @return 拡張フレーム（DLC=3），応答は要求しない
/// @throws std::invalid_argument ID が予約済みの 254 または broadcast の 255
can_msgs::msg::Frame make_reboot_frame(std::uint8_t controller_id);

/// @brief 故障コードのみを含む selective 応答を検査して読み取る
/// @note 宛先は kReplyCanId，送信元 VESC ID は data[0]
/// @return 正しい形式なら true，不正フレームでは出力を変更しない
bool decode_fault_reply(const can_msgs::msg::Frame &frame, FaultCode &fault);

using Status1 = MotionFeedback;
using Status2 = ChargeFeedback;
using Status3 = EnergyFeedback;
using Status4 = ControlFeedback;
using Status5 = PositionFeedback;
using Status6 = AnalogFeedback;

// 指令関数名は公式 firmware 6.06 の comm_can.c に対応
// このライブラリでは CAN フレームを返し，送信は CanInterface が担当する

/// @brief duty 比の指令フレームを生成
/// @param controller_id 宛先 ID（0〜254）
/// @param duty VESC 方向の比率 [-1, 1]，0.5 は 50%
/// @return 拡張 CAN フレーム（DLC=4），比率を 100000 倍して格納
/// @throws std::out_of_range 非有限値または比率の範囲外
can_msgs::msg::Frame comm_can_set_duty(std::uint8_t controller_id, double duty);

/// @brief ERPM の速度指令フレームを生成
/// @param controller_id 宛先 ID（0〜254）
/// @param electrical_rpm 電気回転数 [ERPM]
/// @return 拡張 CAN フレーム（DLC=4）
can_msgs::msg::Frame comm_can_set_rpm(std::uint8_t controller_id, std::int32_t electrical_rpm);

/// @brief モータ電流の指令フレームを生成
/// @param controller_id 宛先 ID（0〜254）
/// @param motor_current_a VESC 方向のモータ電流 [A]
/// @return 拡張 CAN フレーム（DLC=4）
/// @throws std::out_of_range 非有限値または倍率変換後の int32 範囲外
can_msgs::msg::Frame comm_can_set_current(std::uint8_t controller_id, double motor_current_a);

/// @brief モータ PID 角度の指令フレームを生成
/// @param controller_id 宛先 ID（0〜254）
/// @param motor_pid_position_deg モータ PID 角度 [deg]
/// @return 拡張 CAN フレーム（DLC=4）
/// @throws std::out_of_range 非有限値または倍率変換後の int32 範囲外
can_msgs::msg::Frame comm_can_set_pos(std::uint8_t controller_id, double motor_pid_position_deg);

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
/// @brief 共通指令を VESC の CAN フレームへ変換する
can_msgs::msg::Frame encode_command(const VescCommand &command);
/// @brief CAN の STATUS / 故障応答を共通受信値へ変換する
/// @return 未対応・不正フレームでは nullopt
std::optional<VescFeedback> decode_feedback(const can_msgs::msg::Frame &frame);
}  // namespace vesc_can_driver::protocol
