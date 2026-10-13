# vesc_can_ros2_control

`vesc_can` の共通ドライバを利用する、複数関節向け ros2_control SystemInterface です。
このパッケージには hardware plugin、URDF / controller 設定例、control 用 launch、
plugin のテストがあります。初期対応は ROS 2 Lyrical / ros2_control 6 です。
実機では未検証です。

モードごとの指令、フィードバック、ゼロ指令の違いは
[モータの制御モード](../docs/motor-modes.md) を参照してください。

CAN フレームの encode/decode、送受信、通常ノードと模擬 VESC は
[vesc_can](../vesc_can/README.md) にあります。
controller_manager が `vesc_can_ros2_control/VescSystem` をロードし、
電流は current [A]、速度と電流＋速度は velocity [rad/s]、位置は position [rad] を指定します。

同じ CAN ID を通常ノードと hardware plugin から同時に制御しないでください。

## ros2_control の設定

[urdf/vesc_example.urdf](urdf/vesc_example.urdf) は駆動 1 台・操舵 1 台の例です。
実ロボットの URDF に `<ros2_control type="system">` の部分を追加し、
各 VESC を `<joint>` として記述します。4 輪独立駆動・操舵なら同じ System 内へ
8 関節を登録できます。CAN ID は同じバス内で重複させません。

各関節の command_interface は current、velocity、position のいずれかを 1 つ指定します。
モードは joint param の `control_mode` で current / velocity / position / current_velocity を
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
| control_mode | current / velocity / position / current_velocity。省略時は command interface と同じ |
| max_current_a | 電流指令の上限 [モータ A]、既定 10。速度 PID の電流制限設定は VESC 側で行う |
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
activation_timeout_ms (既定 2000) を指定できます。
current state interface は関節方向の符号を適用したモータ電流 [A] です。
電流モードでも position state interface を登録する場合は STATUS5 が必要です。
4 モードの URDF / controller 設定と起動コマンドは
[モータの制御モード](../docs/motor-modes.md#設定例) を参照してください。

速度変換は `ERPM = joint_rad_s × 60/(2π) × gear_ratio × pole_pairs × direction`。
STATUS5 のタコメータは 1 電気回転を 6 カウントとして関節位置へ変換します。
これは起動後の相対位置で、絶対エンコーダではありません。VESC の再起動や
タコメータの int32 境界で位置が飛ぶ場合があります。積算の連続化・リセット検出は未実装です。

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

plugin は `vesc_can::CanTransport` を利用します。送受信トピック、QoS、必要な STATUS、
停止フレームと Zenoh pico の条件は [CAN 接続条件](../vesc_can/README.md#can-と-zenoh-pico-の接続条件)
を参照してください。[構成とスレッド](../docs/architecture.md) に read/write と
通信 executor の境界を記載しています。

## テスト

このパッケージでは hardware 設定の検査と、実際の controller_manager からの
plugin 読み込み、駆動・操舵、フィードバック途絶時の全台停止を確認します。
統合テストは `vesc_can` の通常ノード・模擬 VESC も起動し、標準指令と vesc_msgs の
テレメトリを確認します。
