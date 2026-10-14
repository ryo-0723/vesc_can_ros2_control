# vesc_can_driver

VESC のネイティブ CAN フレームを encode/decode するライブラリと、共通ドライバ、
通常ノード、模擬 VESC を提供します。`ros2_control` には依存しません。
`can_msgs/msg/Frame` の送受信を SocketCAN ブリッジや Zenoh pico の中継器へ接続できます。
初期対応は ROS 2 Lyrical / C++20 です。実機では未検証です。

制御モードの選び方、変換式、ゼロ指令の意味は
[モータの制御モード](../docs/motor-modes.md) を参照してください。

## ライブラリとして使う

| CMake target | ヘッダ | 役割 |
|---|---|---|
| vesc_can_driver::protocol | vesc_can_driver/vesc_protocol.hpp | SET_RPM / SET_CURRENT / SET_POS の encode、STATUS1〜6 の decode |
| vesc_can_driver::driver | vesc_can_driver/vesc_driver.hpp | モータ設定・状態、4 モードの制御、CAN の周期送受信、専用 executor と最新指令の受け渡し |
| vesc_can_driver::state | vesc_can_driver/vesc_state.hpp | VescState への変換とモータごとの VescStateStamped 配信を両入口で共用 |

encode/decode だけを使う場合は protocol target にリンクします。このライブラリは
ROS ノードや executor を起動せず、rclcpp や hardware_interface にリンクしません。

```cmake
find_package(vesc_can_driver REQUIRED)
target_link_libraries(my_can_node PRIVATE vesc_can_driver::protocol)
```

利用側の package.xml には `<depend>vesc_can_driver</depend>` を記載します。

```cpp
#include "vesc_can_driver/vesc_protocol.hpp"

auto command = vesc_can_driver::protocol::make_set_rpm_frame(11, 420);  // ERPM
vesc_can_driver::protocol::Status1 status;
if (vesc_can_driver::protocol::decode_status_1(received_frame, status)) {
  // status.controller_id, status.electrical_rpm, status.motor_current_a, status.duty_cycle
}
```

不正な ID・フラグ・DLC の受信フレームは decode が false を返します。
指令の broadcast ID や表現範囲外の値は例外になります。
ライブラリと型の C++ namespace は `vesc_can_driver`、パケット関数は `vesc_can_driver::protocol` です。
フレーム送信と制御周期は呼び出し側が管理します。

`ready` / `enable` はパケット処理には不要です。`DriverCore` も有効化せずに STATUS を
受信できます。動作の許可・指令期限・全台停止は `vesc_driver` 内で管理します。
通常ノードと hardware plugin は同じ `CanTransport` と `MotorStatePublisher` を使います。
直接 C++ API を使う場合の分担と移行方法は
[ソースの構成](../docs/source-layout.md#ready--enable-とパケット処理の違い) を参照してください。

## 通常ノード

```bash
ros2 launch vesc_can_driver vesc_standalone.launch.py use_mock:=true
# 別端末：フィードバックを受信した後に有効化
ros2 service call /vesc_can_ros2_control/enable std_srvs/srv/SetBool '{data: true}'
ros2 topic pub -r 20 /vesc_can_ros2_control/joint_commands sensor_msgs/msg/JointState \
  '{name: [drive_joint, steer_joint], position: [0.0, 0.25], velocity: [1.0, 0.0]}'
```

通常ノードの例では /can/tx と /can/rx を使います。物理ブリッジが起動している場合は
テスト用 config の CAN トピックを分け、模擬ノードの can_tx_topic/can_rx_topic も
合わせて変更してください。実機起動では use_mock は既定で false です。

既定は無効状態で、全 VESC の新鮮なフィードバック受信後に enable サービスで
有効化します。auto_enable_once=true は最初の接続時だけ自動有効化します。
異常停止後の自動復帰はありません。再有効化後は全モータへ新しい指令を送ります。

| トピック / サービス | 型 | 単位など |
|---|---|---|
| ~/joint_commands | sensor_msgs/msg/JointState | name 指定、velocity [関節 rad/s]、position [関節 rad]。選択したモードの配列を使用 |
| ~/joint_states | sensor_msgs/msg/JointState | 関節 rad / rad/s、effort は空 |
| ~/enable | std_srvs/srv/SetBool | true: 有効化と異常解除、false: 無効化 |
| /diagnostics | diagnostic_msgs/msg/DiagnosticArray | 接続・有効状態・ドライバ異常 |
| ~/motors/<motor_name>/command | std_msgs/msg/Float64 | 電流はモータ A（関節方向の符号）、速度・電流＋速度は関節 rad/s、位置は関節 rad |
| ~/motors/<motor_name>/state | vesc_msgs/msg/VescStateStamped | 上流定義のモータ生値。speed は ERPM、pid_pos_now はモータ PID 角度 [deg] |

電流以外の指令は関節単位です。`Float64` と `JointState` は同じ変換・範囲検査・期限切れ停止を
使用します。NaN/Inf や範囲外の指令は異常停止になります。
`VescStateStamped` は関節単位へ変換する前の VESC 値を配信します。
複数台ではトピック名の motor_name と `state.controller_id` で識別できます。

```bash
ros2 topic pub -r 20 /vesc_can_ros2_control/motors/drive_joint/command \
  std_msgs/msg/Float64 '{data: 1.0}'
ros2 topic echo /vesc_can_ros2_control/motors/drive_joint/state
```

旧独自メッセージ、SetPosition.srv、logical_id 設定は廃止しました。
状態値の取得範囲、未取得値、旧 API の移行は [メッセージ対応表](../docs/messages.md) を参照してください。
通常ノードの CAN 指令は `update_period_ms` (既定 10 ms、各モータ 100 Hz)、
状態・診断は `state_publish_period_ms` (既定 20 ms、50 Hz) で周期的に配信します。
指令コールバックは最新値を渡し、CAN 送信自体は専用 executor のタイマーが行います。
状態の配信周期と VESC 自体の STATUS 送信周期は独立しています。

通常ノードだけの従来設定 max_rpm / rpm_slew_rate はモータ機械 RPM / RPM毎秒です。
pole_pairs の既定は 7。startup_current_a の既定は **0** に変更しており、
直接 RPM 制御します。電流始動は `control_mode: current_velocity` で選択し、
startup_timeout_ms (既定 1000) で打ち切ります。hardware plugin も同じ 4 モードを使います。
電流モードは `control_mode: current`、指令上限は max_current_a (既定 10 A) です。
JointState の effort は電流入力として使いません。
CAN トピックのパラメータ未指定時の既定値は従来の /socketcan_bridge/tx と /rx です。

## CAN と Zenoh pico の接続条件

送信側の Frame は拡張 ID、DLC=4、固定 8 バイト配列の先頭 4 バイトを使用します。
ID は `(packet_id << 8) | controller_id`、値は big endian の符号付き int32。
SET_CURRENT=1 (A×1000)、SET_RPM=3 (ERPM)、SET_POS=4 (deg×1e6) を使用します。
受信は STATUS1=9、STATUS2=14、STATUS3=15、STATUS4=16、STATUS5=27、STATUS6=58。
デコードの基準は VESC firmware 6.06 (`release_6_06` の固定 commit) です。
公式送信形式はすべて DLC=8 です。STATUS5 は末尾 2 バイトが予約領域です。
STATUS5 も DLC=8 のみ受け付け、以前の DLC=6 互換形式は受け付けません。
中継器は元の DLC と全ペイロードを保持してください。
各 STATUS の項目・倍率・参照 commit は
[VESC firmware 6 の STATUS 形式](../docs/messages.md#vesc-firmware-6-の-status-形式) を参照してください。
標準 ID、RTR、エラーフレーム、不一致の DLC、未登録 ID は状態更新に使いません。

VESC Tool で必要な STATUS の定期送信を有効化してください。速度には STATUS1、
操舵位置には STATUS1 と STATUS4、駆動の位置状態には STATUS1 と STATUS5 が必要です。
STATUS2/3/6 は任意の診断・テレメトリで、未受信でも制御の有効化を妨げません。
浮動小数点の状態値は各 STATUS の期限切れで NaN になります。
ネイティブ STATUS1〜6 には VESC の fault code はないため、実機故障コードの取得は
未対応です。VescState の fault_code は -1 (未取得)、ドライバ異常は /diagnostics に配信します。

Tx は Reliable/Volatile、Rx は SensorDataQoS (BestEffort/Volatile)。中継側の QoS を
合わせてください。RMW の ContentFilter に依存せず、受信後に CAN ID を検査します。
中継器が別のシリアライズ方式を使う場合は、その Frame 変換を中継側に実装します。
このパッケージに Zenoh pico の通信コードは含みません。

## 停止動作と運用範囲

必要なフィードバックの期限切れ、指令の期限切れ、不正値、電流始動失敗が発生すると、
同じ System/通常ノード内の全モータへ SET_CURRENT(0 A) を送ります。
ゼロ電流はトルクを解放する指令であり、制動や操舵位置の保持ではありません。
無効状態でも定期的に送信します。周期やスレッド停止中の遅延は保証しません。

通信・ROS プロセスが停止すると停止フレーム自体が届かない場合があります。
中継 MCU 側で「古い指令を捨てる」「指令途絶時に停止する」処理を設け、VESC の
タイムアウトも設定してください。受信時刻で鮮度を判定しており、遅延して届く古い
STATUS の送信元時刻は検証していません。DDS/Zenoh/CAN キューの遅延を含めた
実機検証が必要です。

## テスト

このパッケージのテストは native フレームの既知値、符号、DLC、ID、単位変換、
複数台のフィードバック、タイムアウト、異常の保持・解除、独立 executor、
vesc_msgs の単位と未取得値を確認します。
通常ノードと hardware plugin を実際に起動する統合テストは
`vesc_can_ros2_control` のテストにあります。

`test/support/fake_vesc_node.cpp` は模擬ノードのソースです。`BUILD_TESTING=OFF` でも
`use_mock:=true` のデモで使えるように実行ファイルをインストールします。
fake_vesc_node はプロトコルのテスト用で、物理モデル・ブレーキ・PID・電流制限の
実機動作を再現しません。実機の検証は別途必要です。
