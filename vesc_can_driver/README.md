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
| vesc_can_driver::protocol | vesc_can_driver/vesc_protocol.hpp | 動作指令・故障問い合わせの encode、STATUS1〜6・故障応答の decode |
| vesc_can_driver::driver | vesc_can_driver/vesc_driver.hpp | モータ設定・状態、5 モードの制御、CAN の周期送受信、専用 executor と最新指令の受け渡し |
| vesc_can_driver::state | vesc_can_driver/vesc_state.hpp | VescState への変換とモータごとの VescState 配信を両入口で共用 |

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

`ready` / `activate` はパケット処理には不要です。`DriverCore` も有効化せずに STATUS を
受信できます。動作の許可・指令期限・全台停止は `vesc_driver` 内で管理します。
通常ノードと hardware plugin は同じ `CanTransport` と `MotorStatePublisher` を使います。
直接 C++ API を使う場合の分担と移行方法は
[ソースの構成](../docs/source-layout.md#ready--activate-とパケット処理の違い) を参照してください。

## 通常ノード

通常ノードは ros2_control を使わずにモータを動かす入口です
hardware component を使う場合は `vesc_control.launch.py` から controller_manager を起動し、
`vesc_node` は起動しません

`motors` に登録した複数モータを、1 つの `vesc_node` で管理します
指令の `motor_id` は YAML の `logical_id` で、VESC の `controller_id` と分けて管理します
状態はモータ名ごとのトピックを購読し、上位ノードは CAN ID を持つ必要はありません
モータ名は ros2_control の関節名と揃えられます

```bash
ros2 launch vesc_can_driver vesc_standalone.launch.py use_mock:=true
# 別端末：指令の周期配信を開始
ros2 topic pub -r 20 /vesc/commands vesc_can_msgs/msg/VescCommandArray \
  '{stamp: now, commands: [{motor_id: 1, value: 1.0}, {motor_id: 2, value: 0.25}]}'
```

通常ノードの例では /can/tx と /can/rx を使います。物理ブリッジが起動している場合は
テスト用 config の CAN トピックを分け、模擬ノードの can_tx_topic/can_rx_topic も
合わせて変更してください。実機起動では use_mock は既定で false です。

通常ノードは必要な新鮮な STATUS と指令が揃うと動作を開始します
有効化・無効化サービスや auto_enable_once の設定は使いません
異常停止は保持し、reboot サービスで対象 VESC を再起動してから全モータへ新しい指令を送ります

| トピック / サービス | 型 | 単位など |
|---|---|---|
| /vesc/commands | vesc_can_msgs/msg/VescCommandArray | motor_id は論理 ID、value は起動時モードの単位、stamp は配列の生成時刻 |
| /vesc/event | vesc_can_msgs/msg/VescEvent | 異常を検出した処理から motor_id とエラー文字列を即時通知 |
| ~/reboot | vesc_can_msgs/srv/Reboot | motor_id に論理 ID を指定し、対応する 1 台を再起動 |
| /vesc/state | vesc_can_msgs/msg/VescStateArray | 全モータの状態と配信時刻、各要素の motor_id は指令と共通 |

`VescCommandArray.commands` の各要素は `uint8 motor_id / float32 value` です
配列の順序は `motors` の登録順と違っても使えます
電流モードはモータ A（関節方向の符号）、速度・速度＋電流は関節 rad/s、位置は関節 rad です
duty_cycle は関節方向の比率 [-1, 1] で、0.5 が 50% です
duty の符号は direction、上限は max_duty（既定 1.0）で設定します
制御モードは YAML の control_mode で起動時に固定し、指令に mode は持たせません
通常ノードは control_mode / command_interface パラメータの起動後の変更も拒否します
配列の stamp に生成時の ROS 時刻を入れます
タイムアウトは stamp ではなく、ドライバが受信した単調時刻で判定します
1 台だけでも複数台でも同じ配列型を使い、未指定モータの指令や受信時刻は更新しません
空の配列は指令を更新しません
未登録 ID、ID の重複、NaN/Inf、範囲外の指令は全台停止になります
`command_topic`、`state_topic`、`error_topic` でトピック名を設定できます
指令配列は 1 回のロックと共通の受信時刻で一括保存します
未送信の同じモータへの指令は最新値で上書きし、送信待ちの指令履歴を作りません
同じ値の再受信でも指令期限を更新します

`VescState` は関節速度 `velocity_rad_s` [rad/s]、積算関節角 `position_rad` [rad]、
PID 関節角 `pid_pos_rad` [rad] と監視値を配信します
速度・角度は float64、電流・duty・電荷・電力量・温度・電圧は float32 です
電流と duty は VESC 生値の符号を保持します
`position_rad = tachometer_counts × 2π / (6 × pole_pairs × gear_ratio) × direction` です
全モードで同じ式を使い、2π で折り返さず、未受信・期限切れでは NaN を返します
オドメトリには前回値との差分を使います
STATUS5 を必須にする場合は各モータの `require_tachometer: true` を設定します
VESC 側で STATUS5 の周期送信を有効にし、CAN ブリッジの受信フィルタでも通してください
積算角の測定更新は STATUS5 の受信周期に従い、状態トピックの配信周期とは別です
通常ノードは `/vesc/state` に全モータを YAML の motors 順で配信します
各要素の `motor_id` は指令と同じ logical_id で、上位ノードはこの ID で対象を選びます
配列全体と各要素の stamp は同じ配信時刻で、CAN の同時測定を表しません
`state_topic: ""` を指定すると `~/motors/<motor_name>/state` に個別配信できます
全モータを読む場合は配列で 1 周期 1 回の publish にまとめられます
一部だけ読む場合は個別トピックで必要なモータのデータだけ受け取れます
配列の publisher は KeepLast(1) / Reliable です
publish の呼び出しごとに配信処理へ進むため、全台分を配列にすると呼び出し回数を減らせます
根拠となる処理は [rclcpp Publisher](https://github.com/ros2/rclcpp/blob/lyrical/rclcpp/include/rclcpp/publisher.hpp#L260-L271) を参照してください
CPU や帯域の削減率を実測した結論ではなく、この配信回数に基づく選択です

通常ノードの周期出力は VescStateArray です
異常は検出時だけ `/vesc/event` に通知し、周期の `/diagnostics` は配信しません
関節状態は ros2_control の state interface から取得し、トピックが必要なら
joint_state_broadcaster が `/joint_states` を配信します

```bash
ros2 topic pub -r 20 /vesc/commands vesc_can_msgs/msg/VescCommandArray \
  '{stamp: now, commands: [{motor_id: 1, value: 1.0}]}'
ros2 topic echo /vesc/state
```

旧 ActuatorTarget / ActuatorState、SetPosition.srv は廃止しました
通常ノードの旧 Float64 / JointState / MotorCommands 指令は VescCommandArray に統一しました
指令・状態・エラー・再起動サービスの型は独立した vesc_can_msgs にまとめています
上位パッケージは vesc_can_msgs に依存すればこれらの型を使えます
C++ / Python の利用例は [vesc_can_msgs](../vesc_can_msgs/README.md) を参照してください
状態値の取得範囲、未取得値、旧 API の移行は [メッセージ対応表](../docs/messages.md) を参照してください。
通常ノードの CAN 指令は `update_period_ms` (既定 10 ms、各モータ 100 Hz)、
状態配列は `state_publish_period_ms` (既定 20 ms、50 Hz) で周期的に配信します。
指令コールバックは最新値を渡し、CAN 送信自体は専用 executor のタイマーが行います。
指令受付と状態配信は通常ノードの executor、CAN 受信・フレーム生成・送信は CAN executor です
フレーム生成と CAN トピックへの publish は同じスレッドで順番に行います
指令と状態の作業用 vector は起動時に確保して再利用し、状態取得は try-lock で待機しません
状態の配信周期と VESC 自体の STATUS 送信周期は独立しています。

通常ノードだけの従来設定 max_rpm / rpm_slew_rate はモータ機械 RPM / RPM毎秒です。
pole_pairs の既定は 7。startup_current_a の既定は **0** に変更しており、
直接 RPM 制御します。電流始動は `control_mode: current_velocity` で選択し、
startup_timeout_ms (既定 1000) で打ち切ります。hardware plugin も同じ 5 モードを使います。
電流モードは `control_mode: current`、指令上限は max_current_a (既定 10 A) です。
電流を含む全 5 モードを VescCommandArray で受け付けます
CAN トピックのパラメータ未指定時の既定値は従来の /socketcan_bridge/tx と /rx です。

## rox2026 と同じ形式の YAML

[rox2026.yaml](config/rox2026.yaml) は、rox2026 の `vesc_driver.yaml` と同じ
モータ一覧とモータ名ごとの設定を使う例です
upper_belt は論理 ID 10 / CAN ID 51、under_belt は 11 / 52、dribble は 12 / 50 です
論理 ID は 0〜255 の範囲で重複させず、CAN ID は 0〜254 で重複させません
ros2_control の制御 interface は関節名で管理し、URDF の各 joint にも logical_id を指定します
同じモータの YAML と URDF で logical_id を揃えると、どちらの入口でも同じ motor_id で状態・エラーを扱えます

```bash
ros2 launch vesc_can_driver vesc_standalone.launch.py \
  config:="$(ros2 pkg prefix --share vesc_can_driver)/config/rox2026.yaml"
ros2 topic pub -r 20 /vesc/commands vesc_can_msgs/msg/VescCommandArray \
  '{stamp: now, commands: [{motor_id: 10, value: 20.0}, {motor_id: 11, value: 20.0}, {motor_id: 12, value: 20.0}]}'
```

必要なフィードバックが揃うと、指令だけで動作を開始します
旧 rox2026 の target は機械 RPM ですが、新 API の速度は関節 rad/s です
CAN 指令は 10 ms、状態は 20 ms とし、旧コードの 25 ms / 500 ms から変更しています
状態は /vesc/state の VescStateArray にまとめ、旧 ActuatorStateArray は生成しません

rox2026 の /socketcan_bridge/vesc/rx には STATUS1 と故障応答を通すフィルタを設定します
host_can_id=254 の例は `00000900:9FFFFF00,000008FE:9FFFFFFF` です
STATUS2〜6 も使う場合は各パケット ID のフィルタも追加します

## 即時の異常通知

`/vesc/event` は通常ノードと hardware component 共通の VescEvent です
周期の状態配信を待たず、異常を検出した処理から配信します
内容は uint8 motor_id と string message の 2 項目です
VESC 本体の故障は該当 ID へ `VESC fault_code=4 (ABS_OVER_CURRENT)` のような文字列を通知します
故障名は [VESC 6.06 の enum](https://github.com/vedderb/bldc/blob/94b305ec124b67b0161f2d3d85c4d5b35e48edc5/datatypes.h#L132-L163) に対応し、未知の番号は UNKNOWN と表示します
PC 側の不正指令や通信途絶は、影響する全 ID へ同じ停止理由を通知します
正常コード 0 はイベントにしません
VESC 本体の同じ故障コードは繰り返さず、別のコードまたは正常応答後の再発を通知します
PC 側の同じ異常はラッチ中に繰り返しません
Reliable / TransientLocal / KeepLast(登録モータ数) で直近のイベントを保持します
全台停止時は全 ID の通知を後から取得できます
これは異常の履歴です。本体の現在の故障コードは周期の状態配信で確認します
PC 側の停止保持は通常ノードの reboot または hardware lifecycle の復旧操作で解除します

```bash
ros2 topic echo /vesc/event vesc_can_msgs/msg/VescEvent --qos-durability transient_local
```

故障コードは、VESC 6.06 の [COMM_GET_VALUES_SELECTIVE](https://github.com/vedderb/bldc/blob/94b305ec124b67b0161f2d3d85c4d5b35e48edc5/comm/commands.c#L358-L453)
で bit 15 だけを要求して取得します
CAN_PACKET_PROCESS_SHORT_BUFFER（8）で送り、要求 DLC=7 / 応答 DLC=8 です
USB/UART のシリアル用フレーミングは付けません
既定の fault_poll_period_ms=100 で各モータへ 10 Hz の問い合わせを行い、異常応答の受信時に通知します
本体からの自発的な故障通知ではないため、検出には問い合わせ周期と通信遅延が加わります
VescState.fault_code にも取得値を反映し、未受信・期限切れは -1、正常応答は 0 です
非ゼロを検出すると全台停止し、通常ノードでは reboot サービスで復旧します
ros2_control は lifecycle の復旧操作を使います
応答期限切れで既知の故障を解除することはありません

host_can_id（既定 254）は応答先となる PC の CAN ID です
同じバス上の全 VESC・他の問い合わせ元と重複させず、255 は使用しません
同じバスで通常ノードと hardware component を併用する場合も別の host_can_id が必要です
fault_poll_period_ms は feedback_timeout_ms より短く設定します
fault_poll_period_ms=0 で問い合わせを無効化できます
故障応答がない場合は fault_code=-1 のまま従来の STATUS による制御を継続し、故障なしとは判断しません
検証対象は各 controller_id が独立した VESC に対応する構成です
6.06 の dual-motor 基板は第2モータの応答にも基板の主 ID が付くため、この取得方式の対象外です

CAN の is_error はブリッジが転送する CAN バス異常で、VESC 本体の故障コードとは別です
ブリッジがエラーフレームを転送しない場合は、フィードバックの期限切れで通信異常を検出します

## 故障応答 ID の根拠と再起動コマンド

ID は VESC firmware 6.06 の公式コードから確認しています
[datatypes.h](https://github.com/vedderb/bldc/blob/94b305ec124b67b0161f2d3d85c4d5b35e48edc5/datatypes.h#L1078-L1088)
で CAN_PACKET_PROCESS_SHORT_BUFFER=8 と定義され、
[comm_can_send_buffer](https://github.com/vedderb/bldc/blob/94b305ec124b67b0161f2d3d85c4d5b35e48edc5/comm/comm_can.c#L389-L419)
が `(8 << 8) | 宛先ID` でフレームを生成します
故障応答もこの共通形式を使い、故障専用の CAN パケット ID は追加されません
host_can_id=254（0xFE）の設定から、応答 CAN ID は 0x8FE になります
VESC 本体の送信元 ID は data[0] にあり、受信フィルタには PC の宛先 ID を使います

再起動コマンドは
[COMM_REBOOT=29](https://github.com/vedderb/bldc/blob/94b305ec124b67b0161f2d3d85c4d5b35e48edc5/datatypes.h#L921-L925)
で、[commands.c](https://github.com/vedderb/bldc/blob/94b305ec124b67b0161f2d3d85c4d5b35e48edc5/comm/commands.c#L648-L651)
では NVIC_SystemReset() を呼びます
公式の short buffer 処理と組み合わせると、CAN 経由の送信形式は次のようになります

| フィールド | 内容 |
|---|---|
| 拡張 CAN ID | `(8 << 8) \| VESCのcontroller_id` |
| DLC | 3 |
| data[0] | PC の host_can_id |
| data[1] | 2（コマンドを処理し、応答を要求しない） |
| data[2] | 29（0x1D、COMM_REBOOT） |

例として CAN ID 51（0x33）の VESC、PC ID 254 なら、拡張 ID 0x833 / DLC=3 / data=FE 02 1D です
29 は CAN ID のパケット番号ではなく、short buffer 内のコマンド番号です
通常ノードの ~/reboot は vesc_can_msgs/srv/Reboot を使います
motor_id は YAML の logical_id で、CAN ID へ変換してその 1 台に再起動フレームを 1 回だけ送ります

```bash
ros2 service call /vesc_can_ros2_control/reboot vesc_can_msgs/srv/Reboot '{motor_id: 11}'
```

rox2026.yaml なら logical_id=11 は under_belt / CAN ID 52 です
未登録の論理 ID は success=false とし、再起動フレームを送りません
success=true は要求の受理を表し、本体の再起動完了や通信成功の確認ではありません
要求は CAN executor で次の送信周期に処理し、全台へゼロ電流を送ってから指定した 1 台を再起動します
保存済みの全台の指令と PC 側の異常停止をリセットし、対象の受信状態を未取得へ戻します
500 ms は対象の指令・STATUS・故障応答を受理せず、待機後に届く新鮮なフィードバックを使います
古い指令を自動で再送せず、復旧後は全モータへ新しい指令を送ります
namespace がある場合はサービス名の先頭に追加します
通常ノードの ~/enable と auto_enable_once は廃止しました

ros2_control の activate / deactivate は hardware lifecycle に従う PC 内部の開始・停止です
VESC の有効化・無効化コマンドは送信しません
hardware component を使う場合は通常ノードの reboot サービスを同時起動せず、
controller_manager の lifecycle で通信とコントローラを復旧します

本体の故障原因が残る場合は、再起動しても同じ故障が再発します
VESC 6.06 には停止時間を経過し DRV fault が解消すると故障番号を解除する
[処理](https://github.com/vedderb/bldc/blob/94b305ec124b67b0161f2d3d85c4d5b35e48edc5/motor/mc_interface.c#L2396-L2411)
もあるため、すべての故障で本体の再起動が必要になるわけではありません

## CAN と Zenoh pico の接続条件

動作指令の Frame は拡張 ID、DLC=4、固定 8 バイト配列の先頭 4 バイトを使用します
故障問い合わせは DLC=7、応答は DLC=8 も中継してください
ID は `(packet_id << 8) | controller_id`、値は big endian の符号付き int32。
SET_DUTY=0 (比率×100000)、SET_CURRENT=1 (A×1000)、SET_RPM=3 (ERPM)、SET_POS=4 (deg×1e6) を使用します
SET_DUTY も拡張 ID で、ID の数値が小さい場合も is_extended=true が必要です
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
STATUS1〜6 に含まれない fault_code は、上記の selective 問い合わせで別途取得します

Tx は Reliable/Volatile、Rx は SensorDataQoS (BestEffort/Volatile)。中継側の QoS を
合わせてください。RMW の ContentFilter に依存せず、受信後に CAN ID を検査します。
中継器が別のシリアライズ方式を使う場合は、その Frame 変換を中継側に実装します。
このパッケージに Zenoh pico の通信コードは含みません。

## 停止動作と運用範囲

本体の故障、必要なフィードバックの期限切れ、指令の期限切れ、不正値、電流始動失敗が発生すると、
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
独自 VescState の単位・未取得値・タコメータの鮮度を確認します
通常ノードと hardware plugin を実際に起動する統合テストは
`vesc_can_ros2_control` のテストにあります。

`test/support/fake_vesc_node.cpp` は模擬ノードのソースです。`BUILD_TESTING=OFF` でも
`use_mock:=true` のデモで使えるように実行ファイルをインストールします。
fake_vesc_node はプロトコルのテスト用で、物理モデル・ブレーキ・PID・電流制限の
実機動作を再現しません。実機の検証は別途必要です。
