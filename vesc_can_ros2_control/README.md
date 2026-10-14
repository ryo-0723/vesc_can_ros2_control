# vesc_can_ros2_control

`vesc_can_driver` の共通ドライバを利用する、複数関節向け ros2_control SystemInterface です。
このパッケージには hardware plugin、URDF / controller 設定例、control 用 launch、
plugin のテストがあります。初期対応は ROS 2 Lyrical / ros2_control 6 です。
実機では未検証です。

モードごとの指令、フィードバック、ゼロ指令の違いは
[モータの制御モード](../docs/motor-modes.md) を参照してください。

CAN フレームの encode/decode、送受信、通常ノードと模擬 VESC は
[vesc_can_driver](../vesc_can_driver/README.md) にあります。
controller_manager が `vesc_can_ros2_control/VescSystem` をロードし、
電流は current [A]、速度と電流＋速度は velocity [rad/s]、位置は position [rad] を指定します。

同じ CAN ID を通常ノードと hardware plugin から同時に制御しないでください。

## ros2_control の設定

[urdf/vesc_example.urdf](urdf/vesc_example.urdf) は駆動 1 台・操舵 1 台の例です。
実ロボットの URDF に `<ros2_control type="system">` の部分を追加し、
各 VESC を `<joint>` として記述します。4 輪独立駆動・操舵なら同じ System 内へ
8 関節を登録できます。CAN ID は同じバス内で重複させません。

各関節の command_interface は current、velocity、position、duty_cycle のいずれかを 1 つ指定します
モードは joint param の `control_mode` で current / velocity / position / duty_cycle / current_velocity を
指定します。省略時は command interface と同じモードです。
current_velocity は velocity command interface を使います。
同じ名前の state_interface も必要です。position/velocity/current/
temperature_fet/temperature_motor/input_voltage/duty_cycle の状態を公開できます。
`effort` は未対応です。電流 [A] をトルク [Nm] として扱いません。

| パラメータ | 意味 |
|---|---|
| controller_id | VESC の CAN ID、0〜254、必須。255 のブロードキャストは禁止 |
| pole_pairs | モータの極対数、正整数、必須。14 極なら 7 |
| gear_ratio | モータ回転数 / 関節回転数。20:1 減速なら 20 |
| direction | 関節正方向に対するモータ方向、+1 または -1 |
| control_mode | current / velocity / position / duty_cycle / current_velocity。省略時は command interface と同じ |
| max_current_a | 電流指令の上限 [モータ A]、既定 10。速度 PID の電流制限設定は VESC 側で行う |
| max_duty | duty_cycle の比率上限、0 < max_duty ≤ 1、既定 1.0 |
| startup_current_a | current_velocity の始動電流 [A]、正数を明示指定し max_current_a 以下にする |
| rpm_control_threshold_rpm | current_velocity の移行閾値 [モータ機械 RPM]、既定 1000 |
| startup_timeout_ms | current_velocity の始動期限、既定 1000 ms |
| max_velocity | 駆動関節の最大速度 [rad/s]、既定 20 |
| velocity_slew_rate | 駆動の指令変化率 [rad/s²]、既定 100 |
| zero_offset_rad | 関節ゼロでの VESC PID 角度 [rad]、既定 0 |
| min_position / max_position | 操舵関節の許容範囲 [rad]、既定 -π〜π |
| feedback_timeout_ms | 必要な STATUS の期限、既定 500 ms |
| command_timeout_ms | hardware `write()` / 通常ノード入力の期限、既定 500 ms |

hardware パラメータには can_tx_topic、can_rx_topic、send_period_ms (既定 10)、
state_publish_period_ms (既定 20)、activation_timeout_ms (既定 2000) を指定できます。
current state interface は関節方向の符号を適用したモータ電流 [A] です。
duty_cycle state interface も direction を適用した関節方向の比率です
VescState の電流と duty_cycle は VESC 生値を保持します
電流モードでも position state interface を登録する場合は STATUS5 が必要です。
5 モードの URDF / controller 設定と起動コマンドは
[モータの制御モード](../docs/motor-modes.md#設定例) を参照してください。

速度変換は `ERPM = joint_rad_s × 60/(2π) × gear_ratio × pole_pairs × direction`。
STATUS5 のタコメータは 1 電気回転を 6 カウントとして関節位置へ変換します。
位置制御以外の `position` state interface はこの積算角 [rad] を返し、2π で折り返しません
全モードの `VescState.position_rad` にも同じ換算式の積算角を配信します

```text
position_rad = tachometer_counts × 2π / (6 × pole_pairs × gear_ratio) × direction
```

オドメトリ controller は車輪の `position` state interface を直接取得できます
車輪半径を r とすると、前回値からの移動距離は `Δs = r × (position_now − position_prev)` です
初回は現在値を基準として保存し、左右輪などの差分からロボットの移動量を求めます
必要な URDF 設定は次の通りです（VescSystem の各 joint 内）

```xml
<param name="control_mode">velocity</param>
<param name="pole_pairs">7</param>
<param name="gear_ratio">1.0</param>
<param name="direction">1</param>
<command_interface name="velocity"/>
<state_interface name="position"/>
<state_interface name="velocity"/>
```

この構成では STATUS5 が必須となり、未受信・期限切れでは有効化または制御を継続しません
VESC の STATUS5 周期送信と CAN ブリッジの受信を有効にしてください
state interface は 100 Hz で読み取り、測定値そのものの更新周期は STATUS5 の受信周期に従います
積算角は VESC のタコメータ原点に基づく相対位置です。VESC の再起動や
タコメータの int32 境界で位置が飛ぶ場合があります。積算の連続化・リセット検出は未実装です。
再起動後はオドメトリ側の前回値を取り直して基準を合わせます

位置制御モードの `position` state interface は STATUS4 を維持します
操舵は `SET_POS` と STATUS4 の PID 角度を使用します。モータ側の単回転角度を
関節角度へ変換するため、現在は許容範囲を `[-π/gear_ratio, π/gear_ratio]` 内に
限定します。多回転減速機の出力軸の絶対位置やホーミングは未対応です。
VESC Tool のエンコーダ設定、PID position angle division、ゼロ位置の校正によって
角度の意味が変わるため、実機の読み取り値と変換式の一致を先に確認してください。

```bash
# 模擬 VESC を起動してプラグインとコントローラを確認
ros2 launch vesc_can_ros2_control vesc_control.launch.py use_mock:=true \
  can_tx_topic:=/bench/can/tx can_rx_topic:=/bench/can/rx
ros2 topic pub -r 20 /drive_controller/commands std_msgs/msg/Float64MultiArray '{data: [1.0]}'
ros2 topic pub -r 20 /steer_controller/commands std_msgs/msg/Float64MultiArray '{data: [0.25]}'
```

実機では use_mock を指定せず、実機用 urdf と controllers_file を渡します。
付属の forward_command_controller は最後の値を保持します。上位トピックの指令が
途絶えても hardware `write()` が同じ値を毎周期書けば、hardware のタイムアウトは
発生しません。実機用コントローラには上位指令のタイムアウトを設けてください。

## 通信と停止

hardware component 使用時は controller_manager が `VescSystem` を読み込みます
`vesc_control.launch.py` は通常ノード `vesc_node` を起動しません
VESC の状態は共通 driver が保持し、hardware の read() が state interface へ反映します
`/joint_states` は joint_state_broadcaster が配信します

plugin は `vesc_can_driver::CanTransport` を利用します。送受信トピック、QoS、必要な STATUS、
停止フレームと Zenoh pico の条件は [CAN 接続条件](../vesc_can_driver/README.md#can-と-zenoh-pico-の接続条件)
を参照してください。[構成とスレッド](../docs/architecture.md) に read/write と
通信 executor の境界を記載しています。

## 更新と状態配信の周期

| 処理 | 同梱設定 | 設定箇所 |
|---|---|---|
| CAN 指令の送信（各モータ） | 100 Hz | hardware の `send_period_ms: 10` |
| state interface の更新・controller 実行 | 100 Hz | controller_manager の `update_rate: 100` |
| `/joint_states` の配信 | 50 Hz | joint_state_broadcaster の `update_rate: 50` |
| 各モータの `VescState` の配信 | 50 Hz | hardware の `state_publish_period_ms: 20` |

state interface を直接読むオドメトリ controller は 100 Hz の更新を使えます。
`VescState` は CAN 用 executor の状態タイマーで配信し、read/write では生成・配信しません。
配信先は `/<ros2_control の name>_can_io/motors/<joint名>/state` です。
例の `VescFiveModes` なら `/VescFiveModes_can_io/motors/drive_joint/state` になります。
namespace を付けた場合はこの先頭に追加されます。電流・温度・電圧などの生値と単位は
通常ノードと同じ [メッセージ対応表](../docs/messages.md) に従います。

ドライバが異常を検出すると、周期の状態配信とは別に `/vesc/event` へ
DiagnosticArray を即時配信します
hardware の `error_topic` param で配信先を変更できます
read/write は ROS 通知を配信せず、CAN executor が異常を処理した直後に通知します
VESC 本体の fault_code は CAN selective 問い合わせで取得します
hardware の host_can_id（既定 254）と fault_poll_period_ms（既定 100）で設定します
受信時に /vesc/event へ source=vesc / controller_id / fault_code を載せて通知し、全台停止します
hardware の停止保持は controller_manager の lifecycle 復旧で解除します
activate / deactivate は PC 内部の状態で、VESC 有効化・無効化パケットは送りません
VescState.fault_code は未受信・期限切れに -1 を返します
応答先 ID と CAN ブリッジの受信フィルタは [ドライバの説明](../vesc_can_driver/README.md#即時の異常通知) を確認してください

## テスト

このパッケージでは hardware 設定の検査と、実際の controller_manager からの
plugin 読み込み、駆動・操舵、フィードバック途絶時の全台停止を確認します。
統合テストでは、別の CAN トピックを使う通常ノード・模擬 VESC も起動し、
通常ノードの VescCommandArray 指令と VescStateArray の状態出力を確認します
通常ノードを hardware component の通信経路として使う構成ではありません
