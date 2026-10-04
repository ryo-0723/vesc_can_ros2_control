# vesc_can_ros2_control

VESC のネイティブ CAN 指令を `can_msgs/msg/Frame` で送受信します。
USB/UART に依存せず、SocketCAN ブリッジや Zenoh pico の中継器へ接続できます。
初期対応は ROS 2 Lyrical、ros2_control 6 です。実機では未検証です。

## 2 種類の使い方

| 使い方 | 起動するもの | 指令と状態 |
|---|---|---|
| ros2_control | controller_manager が `vesc_can_ros2_control/VescSystem` をロード | 関節の velocity [rad/s] または position [rad] |
| 通常ノード | `vesc_node` | 標準 JointState トピック、または既存の独自メッセージ |
| 模擬 VESC | `fake_vesc_node` | CAN 指令を受信して STATUS1/4/5 を返すテスト用ノード |

同じ CAN ID を制御する通常ノードと hardware plugin は同時に起動しないでください。
停止中もゼロ電流フレームを定期送信するため、同じ ID への送信者は 1 つにします。

## ros2_control の設定

[urdf/vesc_example.urdf](urdf/vesc_example.urdf) は駆動 1 台・操舵 1 台の例です。
実ロボットの URDF に `<ros2_control type="system">` の部分を追加し、
各 VESC を `<joint>` として記述します。4 輪独立駆動・操舵なら同じ System 内へ
8 関節を登録できます。CAN ID は同じバス内で重複させません。

各関節の command_interface は velocity または position を 1 つ指定します。
同じ名前の state_interface も必要です。position/velocity/current/
temperature_fet/temperature_motor/input_voltage/duty_cycle の状態を公開できます。
`effort` は未対応です。電流 [A] をトルク [Nm] として扱いません。

| パラメータ | 意味 |
|---|---|
| controller_id | VESC の CAN ID、0〜254、必須。255 のブロードキャストは禁止 |
| pole_pairs | モータの極対数、正整数、必須。14 極なら 7 |
| gear_ratio | モータ回転数 / 関節回転数。20:1 減速なら 20 |
| direction | 関節正方向に対するモータ方向、+1 または -1 |
| max_velocity | 駆動関節の最大速度 [rad/s]、既定 20 |
| velocity_slew_rate | 駆動の指令変化率 [rad/s²]、既定 100 |
| zero_offset_rad | 関節ゼロでの VESC PID 角度 [rad]、既定 0 |
| min_position / max_position | 操舵関節の許容範囲 [rad]、既定 -π〜π |
| feedback_timeout_ms | 必要な STATUS の期限、既定 500 ms |
| command_timeout_ms | hardware `write()` / 通常ノード入力の期限、既定 500 ms |

hardware パラメータには can_tx_topic、can_rx_topic、send_period_ms (既定 10)、
activation_timeout_ms (既定 2000) を指定できます。

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

## 通常ノード

```bash
ros2 launch vesc_can_ros2_control vesc_standalone.launch.py use_mock:=true
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
| /vesc/target, /vesc/target_array | vesc_can_interfaces/msg/ActuatorTarget, ActuatorTargetArray | logical_id 指定、駆動はモータ機械 RPM、操舵は関節 rad |
| /vesc/state, /vesc/state_array | vesc_can_interfaces/msg/ActuatorState, ActuatorStateArray | velocity はモータ機械 RPM、position は関節 rad、torque_nm は NaN |

旧速度入力は範囲内へクランプします。JointState と ros2_control では範囲外や
NaN/Inf の指令を異常として停止します。独自メッセージの定義は変更していません。
SetPosition.srv は定義を保持していますが、このノードに対応するサービスはありません。

通常ノードだけの従来設定 max_rpm / rpm_slew_rate はモータ機械 RPM / RPM毎秒です。
pole_pairs の既定は 7。startup_current_a の既定は **0** に変更しており、
直接 RPM 制御します。旧方式の電流始動は明示設定で利用でき、startup_timeout_ms
(既定 1000) で打ち切ります。hardware plugin は電流始動を使いません。
CAN トピックのパラメータ未指定時の既定値は従来の /socketcan_bridge/tx と /rx です。

## CAN と Zenoh pico の接続条件

送信側の Frame は拡張 ID、DLC=4、固定 8 バイト配列の先頭 4 バイトを使用します。
ID は `(packet_id << 8) | controller_id`、値は big endian の符号付き int32。
SET_CURRENT=1 (A×1000)、SET_RPM=3 (ERPM)、SET_POS=4 (deg×1e6) を使用します。
受信は STATUS1=9 (DLC=8)、STATUS4=16 (DLC=8)、STATUS5=27 (DLC=6)。
標準 ID、RTR、エラーフレーム、不一致の DLC、未登録 ID は状態更新に使いません。

VESC Tool で必要な STATUS の定期送信を有効化してください。速度には STATUS1、
操舵位置には STATUS1 と STATUS4、駆動の位置状態には STATUS1 と STATUS5 が必要です。
STATUS4/5 の任意の温度・電圧状態は期限切れで NaN になります。
ネイティブ STATUS1/4/5 には VESC の fault code はないため、実機故障コードの取得は
未対応です。独自 state の fault_code=1 はドライバ異常で、VESC 故障番号ではありません。

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

colcon test はネイティブフレームの既知値、符号、DLC、ID、単位変換、2 台の状態、
タイムアウト、異常の保持・解除、独立 executor、実際の controller_manager からの
プラグイン読み込みと駆動/操舵操作、通常ノードの旧/標準トピック操作を確認します。
fake_vesc_node はプロトコルのテスト用で、物理モデル・ブレーキ・PID・電流制限の
実機動作を再現しません。実機の検証は別途必要です。
