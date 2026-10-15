# モータの制御モード

この文書は本リポジトリの実装が受け付ける設定と指令を説明します。
VESC 自体の全制御機能を提供する API ではありません。モードは関節ごとに起動時に
選択し、速度・電流・位置・duty 比・速度＋電流を同じ System / 通常ノード内で混在できます。
実行中のモード切り替えは実装していません。設定変更後は通常ノードを再起動するか、
URDF を変更して hardware を再ロードします。

| モード | 主な用途 | ROS 指令 | CAN 指令 | 有効化・継続動作に必要な状態 |
|---|---|---|---|---|
| `current` | モータ電流の直接指令 | モータ A、正負は関節方向 | `SET_CURRENT` | STATUS1。位置状態が必要なら STATUS5 も必要 |
| `velocity` | 車輪などの回転速度 | 関節 rad/s | `SET_RPM`、値は ERPM | STATUS1。位置状態が必要なら STATUS5 も必要 |
| `position` | 操舵などの角度 | 関節 rad | `SET_POS`、値はモータ PID 角度 deg | STATUS1 と STATUS4。位置が設定範囲内であること |
| `duty_cycle` | PWM duty 比の直接指令 | 比率 [-1, 1]、正負は関節方向 | `SET_DUTY`、比率×100000 | STATUS1。位置状態が必要なら STATUS5 も必要 |
| `current_velocity` | 電流で始動してから速度制御 | 関節 rad/s | 始動中 `SET_CURRENT`、移行後 `SET_RPM` | STATUS1。位置状態が必要なら STATUS5 も必要 |

`effort` の指令は未対応です
電流 [A] をトルク [Nm] として扱いません
モード選択・構成検査・指令の範囲検査は共通ドライバの関数を使います。
電流モード、電流始動、全台停止は同じ電流フレーム生成処理を使い、速度モードと
電流＋速度の移行後は同じ変化率制限・ERPM 変換処理を使います。

## 電流モード

`max_current_a` 以下のモータ電流を直接指令します。既定の指令上限は 10 A で、
機種・用途に合わせて明示設定します。正負は関節の正方向に合わせます。

```text
CAN current [A] = command [A] × direction
```

例えば `direction=-1` で +1.5 A を指定すると `SET_CURRENT(-1.5 A)` を送ります。
電流は減速比・極対数で倍率を変えません。電流指令に速度の変化率制限は適用しません。
正負の上限は対称で、上限を超えた指令は全台の異常停止です。
0 A の指令は電流解放です。

ros2_control の `current` 状態も `direction` を適用したモータ A です。
以前はこの state interface が VESC 生値の符号でした。`direction=-1` の利用者は
今回から状態の符号も反転する点を確認してください。
通常ノードの `VescState.motor_current_a` は VESC 生値の A を保持します。
これはモータ電流で、トルク定数や減速機効率を使うトルク換算は行いません。
`max_current_a` は本ドライバが送る電流指令の上限で、速度 PID が出す電流の制限を
VESC に設定する機能ではありません。実際の電流制限は VESC 側にも設定します。

## 速度モード

```text
motor_RPM = joint_rad_s × 60 / (2π) × gear_ratio × direction
ERPM      = motor_RPM × pole_pairs
```

`gear_ratio` はモータ回転数 / 関節回転数、`pole_pairs` は極対数です。
14 極のモータなら `pole_pairs=7`。減速比 20:1 なら `gear_ratio=20`。
`direction=-1` は指令と関節フィードバックの正負を両方反転します。
例えば `gear_ratio=2`、`pole_pairs=7`、`direction=1` で π rad/s を指令すると
モータは 60 RPM、CAN 指令は 420 ERPM になります。

非ゼロ指令は変化率を制限し、ERPM を整数に丸めて送信します。
最大速度を超える指令は上限へ丸めず、異常として同じドライバの全台を停止します。
**速度 0 の指令は `SET_CURRENT(0 A)` を即時に選択します。**
速度 PID による停止保持やブレーキではなく、変化率制限も通りません。

| 設定対象 | 最大速度 | 指令変化率 |
|---|---|---|
| ros2_control の joint param | `max_velocity` [関節 rad/s] | `velocity_slew_rate` [関節 rad/s²] |
| 通常ノードの motor param | `max_rpm` [モータ機械 RPM] | `rpm_slew_rate` [モータ機械 RPM/s] |

通常ノードでも ROS トピックに送る速度の単位は関節 rad/s です。
`max_rpm` は ERPM ではありません。
STATUS5 からの積算関節角は、全モードの VescState.position_rad に次の式で配信します

```text
joint_rad = tachometer_counts × 2π / (6 × pole_pairs × gear_ratio) × direction
```

これは VESC タコメータ由来の位置で、起動時の自動ゼロ合わせや絶対位置取得は
行いません。角度を 2π で折り返さず、逆回転では符号付きのカウントに従って減少します
zero_offset_rad は位置 PID 専用で、この積算角には適用しません
再起動・int32 カウンタの折り返しを連続化する処理はありません
plugin は位置モード以外の関節に position state interface を登録すると STATUS5 を必須にします。
通常ノードでは `require_tachometer: true` で必須にします。
状態配信は両入口で 50 Hz、hardware の state interface は controller_manager の 100 Hz 設定で更新します

## 位置モード

```text
motor_deg = (zero_offset_rad + joint_rad × gear_ratio × direction) × 180 / π
CAN 指令  = motor_deg を [0, 360) に折り返した値
joint_rad = remainder(feedback_deg × π / 180 - zero_offset_rad, 2π)
            × direction / gear_ratio
```

`zero_offset_rad` は関節ゼロに対応するモータ PID 角度をモータ側 rad で指定します。
例えば `gear_ratio=2`、`direction=-1`、`zero_offset_rad=π/2` では、
関節 π/4 rad の指令がモータ 0 deg に対応します。
フィードバックには STATUS4 の PID 角度を使います。

`min_position` / `max_position` は関節 rad です。設定は
`-π/gear_ratio ≤ min_position < max_position ≤ π/gear_ratio` に限定します。
例えば 20:1 の減速機では約 ±0.157 rad の範囲内です。
単回転角度からの変換なので多回転位置制御・ホーミングはできません。
±π の折り返し境界では角度の符号が一意にならないため、実用の可動範囲は境界の内側に
設定してください。VESC 側のエンコーダ・PID 角度設定とゼロ位置は実機で校正します。

**位置 0 の指令は関節ゼロへ移動する `SET_POS` です。停止・無効化ではありません。**
位置指令には速度モードの slew rate を適用しません。速度・加速度を制限する軌道は
上位 controller 側で生成します。


## duty 比モード

指令は百分率でなく [-1, 1] の比率です
0.5 は正方向の 50%、-0.25 は逆方向の 25% を表します

```text
CAN duty ratio = command × direction
payload        = round(CAN duty ratio × 100000)
CAN ID         = (0 << 8) | controller_id
```

[公式 VESC 6.06 の comm_can_set_duty](https://github.com/vedderb/bldc/blob/94b305ec124b67b0161f2d3d85c4d5b35e48edc5/comm/comm_can.c#L469-L475) の ID・倍率・符号付き int32 形式を使用します
ペイロードは big endian の 4 バイト、拡張 CAN フレームです
direction=-1、value=0.25 なら比率 -0.25、ペイロード -25000 を送ります
減速比・極対数で倍率を変えず、速度の変化率制限も掛けません

max_duty は絶対値の上限で、0 < max_duty ≤ 1、既定値は 1.0 です
設定の上限を超える値や NaN/Inf は全台の異常停止になります
0 の指令は他の非位置モードと同じ SET_CURRENT(0 A) で電流を解放します

ros2_control の duty_cycle command / state interface は関節方向の比率です
state interface は STATUS1 の duty に direction を適用します
VescState.duty_cycle は VESC 生値の比率を保持します

## 設定例

通常ノードでは YAML の各モータに `control_mode` を指定します
control_mode と command_interface は読み取り専用で、起動後の変更要求は拒否します
`command_interface` は省略時にそのモードから決まります。
両方指定した場合は次の対応表と一致させます。
`control_mode` を省略した旧設定も利用でき、その場合は `command_interface` から
モードを決めます。両方省略すると `velocity` です。

| control_mode | command_interface | 通常ノードの入力 |
|---|---|---|
| current | current | VescCommand.value [A] |
| velocity | velocity | VescCommand.value [rad/s] |
| position | position | VescCommand.value [rad] |
| duty_cycle | duty_cycle | VescCommand.value [比率 -1〜1] |
| current_velocity | current_velocity | VescCommand.value [rad/s] |

```yaml
drive_joint:
  logical_id: 1
  controller_id: 11
  control_mode: velocity
  pole_pairs: 7.0
  gear_ratio: 1.0
  max_rpm: 190.0
  rpm_slew_rate: 955.0
steer_joint:
  logical_id: 2
  controller_id: 21
  control_mode: position
  pole_pairs: 7.0
  gear_ratio: 1.0
  min_position: -1.0
  max_position: 1.0
current_joint:
  logical_id: 3
  controller_id: 12
  control_mode: current
  pole_pairs: 7.0
  max_current_a: 3.0
duty_joint:
  logical_id: 5
  controller_id: 41
  control_mode: duty_cycle
  pole_pairs: 7.0
  direction: -1.0
  max_duty: 0.5
hybrid_joint:
  logical_id: 4
  controller_id: 31
  control_mode: current_velocity
  pole_pairs: 7.0
  max_current_a: 3.0
  startup_current_a: 2.0
  rpm_control_threshold_rpm: 50.0
  startup_timeout_ms: 1000
```

上記は `ros__parameters` 内の抜粋です。完全な設定は
[five_modes.yaml](../vesc_can_driver/examples/five_modes.yaml) を参照してください。
通常ノードの指令は `VescCommandArray.commands` に各モータの指令を入れます
motor_id は YAML の logical_id で、制御モードは起動時の control_mode から決まります
配列の stamp は生成時の ROS 時刻で、指令に mode は持たせません
value に電流 [A]、関節位置 [rad]、関節速度 [rad/s]、duty 比を指定します
上位ノードに CAN ID の対応表を持たせる必要はありません

ros2_control では各 joint に `control_mode` param と上表の command interface を
1 つ記述し、同名の state interface も登録します。
設定例は [vesc_five_modes.urdf](../vesc_can_ros2_control/examples/vesc_five_modes.urdf) と
[controllers_five_modes.yaml](../vesc_can_ros2_control/examples/controllers_five_modes.yaml) です。
電流の forward controller は `interface_name: current`、電流＋速度は
`interface_name: current_velocity`、duty は `interface_name: duty_cycle` を指定します

```bash
ros2 launch vesc_can_ros2_control vesc_control.launch.py use_mock:=true \
  urdf:="$(ros2 pkg prefix --share vesc_can_ros2_control)/examples/vesc_five_modes.urdf" \
  controllers_file:="$(ros2 pkg prefix --share vesc_can_ros2_control)/examples/controllers_five_modes.yaml" \
  controllers:="joint_state_broadcaster drive_controller steer_controller current_controller hybrid_controller duty_controller" \
  can_tx_topic:=/bench/can/tx can_rx_topic:=/bench/can/rx
# 別端末で各指令を周期送信（電流は A、駆動・hybrid は rad/s、操舵は rad、duty は比率）
ros2 topic pub -r 20 /drive_controller/commands std_msgs/msg/Float64MultiArray '{data: [2.0]}'
ros2 topic pub -r 20 /steer_controller/commands std_msgs/msg/Float64MultiArray '{data: [0.25]}'
ros2 topic pub -r 20 /current_controller/commands std_msgs/msg/Float64MultiArray '{data: [1.5]}'
ros2 topic pub -r 20 /hybrid_controller/commands std_msgs/msg/Float64MultiArray '{data: [2.0]}'
ros2 topic pub -r 20 /duty_controller/commands std_msgs/msg/Float64MultiArray '{data: [0.25]}'
```

通常ノードで模擬 5 台を使う場合は、次を別々の端末で起動します。
この構成では模擬ノードを手動で起動し、通常ノード用 launch の use_mock は省略します。

```bash
ros2 run vesc_can_driver fake_vesc_node --ros-args \
  -p controller_ids:='[11, 21, 12, 31, 41]' \
  -p can_tx_topic:=/bench/can/tx -p can_rx_topic:=/bench/can/rx
ros2 launch vesc_can_driver vesc_standalone.launch.py \
  config:="$(ros2 pkg prefix --share vesc_can_driver)/examples/five_modes.yaml"
```

通常ノードでは次の指令ストリームを起動します
必要なフィードバックと新しい指令が揃うと動作を開始します

```bash
ros2 topic pub -r 20 /vesc/commands vesc_can_msgs/msg/VescCommandArray \
  '{stamp: now, commands: [{motor_id: 1, value: 2.0}, {motor_id: 2, value: 0.25}, {motor_id: 3, value: 1.5}, {motor_id: 4, value: 2.0}, {motor_id: 5, value: 0.25}]}'
```

全モータへ command_timeout_ms 内に新しい指令を送り続けます

## 速度＋電流モード

通常ノードと hardware plugin の両方で利用できます。
`startup_current_a > 0`、`rpm_control_threshold_rpm > 0`、
`startup_current_a ≤ max_current_a` を必須とします。まず回転方向に合わせた一定電流を送り、
指令と同じ方向の実測速度が `min(目標機械 RPM, 閾値 RPM)` に達すると速度制御へ
移ります。`startup_timeout_ms` 内に達しなければ全台を異常停止します。
同じ方向の指令の再送で始動期限は延長されません。
速度制御へ移った後、速度が下がっただけでは電流始動へ戻りません。
反転指令、ゼロ後の再指令、再有効化では始動状態をリセットします。
移行時は実測速度を設定上限内に収めてから速度モードと共通の変化率制限を使います。

速度モードは常に直接 RPM 制御します。以前の「velocity に非ゼロの
startup_current_a を設定して暗黙に電流始動する」設定は、
`control_mode: current_velocity` を追加して移行してください。
他の 4 モードで非ゼロの `startup_current_a` を設定すると構成エラーになります。

## 有効化、異常、停止

通常ノードは全台の必要な STATUS と新しい指令が揃うと動作を開始します
VESC の有効化コマンドや enable サービスは使いません
plugin は activate 時にフィードバックを待ち、電流・速度・duty 比・速度＋電流の初期指令を 0、
位置の初期指令を現在位置に設定します。

指令の期限切れ、必要な STATUS の期限切れ、NaN/Inf、範囲外指令、始動失敗で
同じドライバ内の全台へ `SET_CURRENT(0 A)` を送ります。
位置モードも異常停止時は位置保持を解除します。
異常は保持され、フィードバックや指令が再開しても自動復帰しません。
通常ノードは ~/reboot に論理 motor_id を指定して対象 VESC を再起動し、全台へ新しい指令を送ります
plugin は controller_manager による lifecycle の復旧操作で再開します

plugin の `command_timeout_ms` は hardware `write()` の指令更新を監視します。
付属の forward controller は最後の値を保持するので、上位トピックの途絶だけでは
このタイムアウトは発生しません。上位指令の watchdog は controller 側にも必要です。
ROS プロセスや通信そのものが停止すると停止指令が届かない場合があるため、
実機では VESC・中継器のタイムアウトも設定します。
