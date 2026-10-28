# vesc_can_ros2_control

複数のVESCを、1つのros2_control SystemInterfaceで管理します
controller_managerが `vesc_can_ros2_control/VescSystem` を読み込み、通常ノードは起動しません
CAN通信と状態配信には `vesc_can_driver` を使います
CAN送受信、周期処理、故障応答の確認はVescTransportが担当します
各Systemが通信スレッドとpublisherを所有し、cleanupで自分の通信だけを終了します

## 設定例

| URDF | コントローラのYAML | 内容 |
|---|---|---|
| [vesc_example.urdf](examples/vesc_example.urdf) | [controllers.yaml](examples/controllers.yaml) | 駆動の速度制御、操舵の位置制御 |
| [vesc_five_modes.urdf](examples/vesc_five_modes.urdf) | [controllers_five_modes.yaml](examples/controllers_five_modes.yaml) | 5モードを同じSystemで使用 |
| [vesc_prismatic.urdf](examples/vesc_prismatic.urdf) | [controllers_prismatic.yaml](examples/controllers_prismatic.yaml) | 直動と回転の速度制御を混在 |
| [vesc_prismatic_position.urdf](examples/vesc_prismatic_position.urdf) | [controllers_prismatic_position.yaml](examples/controllers_prismatic_position.yaml) | PID・加減速制限付き直動位置制御 |

基本例の起動手順は [ルートREADME](../README.md#ros2_controlで使う)、5モードの起動手順は [モードの説明](../docs/motor-modes.md#設定例) にあります
自分の設定は次のように指定します

```bash
ros2 launch vesc_can_ros2_control vesc_control.launch.py \
  urdf:=/path/to/robot.urdf controllers_file:=/path/to/controllers.yaml
```

## jointの設定

各jointにcommand interfaceを1つ指定します
モードは `control_mode` で選び、省略した場合はcommand interfaceから決まります
速度制御は `velocity`、速度＋電流制御は `current_velocity` command interfaceを使います
どちらも速度のstate interfaceは `velocity`、回転 joint の指令・フィードバックの単位はrad/sです

| 設定 | 意味 |
|---|---|
| logical_id | ROS内のモータ識別子、0〜255で一意にする、必須 |
| motor_id | VESCのCAN ID、0〜253で一意にする、必須 |
| pole_pairs | 極対数、正整数、必須 |
| gear_ratio / direction | モータ回転数÷関節回転数、正方向は+1か-1 |
| control_mode | velocity / current / position / duty_cycle / current_velocity |
| max_velocity / velocity_slew_rate | 関節速度[rad/s]の上限と変化率[rad/s²] |
| max_current_a / max_duty | 電流[A]、duty比の指令上限 |
| min_position / max_position | 位置制御の許容範囲[rad] |
| zero_offset_rad | 関節ゼロに対応するVESCのPID角[rad] |
| feedback_timeout_ms / command_timeout_ms | フィードバックとhardware writeの期限 |

同じモータを通常ノードで使う場合は、YAMLとURDFのlogical_idを揃えます
ホストIDの設定は不要です。CAN ID 254は故障問い合わせの応答先に予約しています
指令に対応するstate interfaceも登録します（`current_velocity` の場合は `velocity`）
追加の状態には `position`、`velocity`、`current`、`duty_cycle`、`temperature_fet`、`temperature_motor`、`input_voltage` を指定できます

車輪のオドメトリに使う設定例です

```xml
<joint name="drive_joint">
  <param name="logical_id">1</param>
  <param name="motor_id">11</param>
  <param name="pole_pairs">7</param>
  <command_interface name="velocity"/>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
</joint>
```

位置制御以外の `position` stateは、STATUS5から求めた積算角です
車輪の移動距離は `車輪半径 × (今回の角度 − 前回の角度)` で求められます
回転 joint の位置制御の `position` stateは、STATUS4のPID角を関節単位へ変換します
VESCの再起動後は、オドメトリ側の前回値を取り直してください
角度の範囲と校正は [モードの説明](../docs/motor-modes.md) を参照してください

## 直動 joint（prismatic）

ロボット本体の URDF にある `joint type="prismatic"` を読み取り、hardware 内で
指令・状態を換算します。`ros2_control` 内の joint 名を本体の joint 名と揃えます。
回転 joint のインターフェースと設定値の単位は従来どおりです。
直動は `velocity` / `current_velocity` と、下記のホスト側 PID による `position` モードに対応します。
`current` / `duty_cycle` の直接指令は直動では拒否します。

| 直動 joint の設定 | 意味 |
|---|---|
| linear_lead_m_per_rev | 減速機出力軸 1 回転あたりの移動距離 [m]、正の有限値、必須 |
| gear_ratio | モータ回転数 ÷ 減速機出力軸回転数、既定値 1 |
| direction | 直動の正方向に合わせた +1 / -1、既定値 +1 |
| position_offset_m | VESC の積算カウント 0 に対応する位置 [m]、既定値 0 |
| max_velocity | 直動速度上限 [m/s]、正の有限値、必須 |
| velocity_slew_rate | 直動速度の変化率上限 [m/s²]、正の有限値、必須 |

ボールねじなら `linear_lead_m_per_rev` はねじのリード、ベルトならプーリ 1 回転で
進む距離です。減速比と方向はドライバが適用し、hardware はその出力軸の角度・速度を
`L / (2π)` 倍します。位置だけに `position_offset_m` を加えます。
例えばリード 0.005 m、減速比 2、方向 -1、極対数 7 なら、+0.005 m/s は -840 ERPM です。

`position` state は STATUS5 の積算値から求める m、`velocity` state と速度指令は m/s です。
`current` など他の state の単位・方向は回転 joint と同じです。
診断トピックの `VescState.position_rad` / `velocity_rad_s` は減速比・方向適用後の
出力軸の rad / rad/s のまま配信します。直動のオフセットも診断トピックには適用しません。

速度モードでは、URDF の位置制限を書くだけでは hardware は速度指令による
ストローク超過を検出しません。`min_position` / `max_position` は直動の位置モード専用です。
`zero_offset_rad` は直動では使用できません。
`position_offset_m` は固定の換算値で、原点復帰や起動時の自動ゼロ合わせではありません。
VESC 再起動・カウンタの折り返しを連続化する処理もありません。
速度モードの速度 0 は従来どおりゼロ電流であり、停止位置を保持する指令ではありません。

模擬 CAN を使った設定例の起動方法です。

```bash
ros2 launch vesc_can_ros2_control vesc_control.launch.py \
  use_mock:=true \
  urdf:=$(ros2 pkg prefix --share vesc_can_ros2_control)/examples/vesc_prismatic.urdf \
  controllers_file:=$(ros2 pkg prefix --share vesc_can_ros2_control)/examples/controllers_prismatic.yaml \
  controllers:="joint_state_broadcaster velocity_controller"
```

`/velocity_controller/commands` に `Float64MultiArray` を送り、配列を
`[slider_joint の m/s, rotary_joint の rad/s]` とします。
例えば `[0.005, 1.0]` なら直動は 5 mm/s、回転は 1 rad/s です。
付属の模擬 VESC はプロトコルの確認用で、機構の端や物理挙動を再現しません。

## 直動 joint の位置制御

`prismatic` の `position` command interface（指令は m）を使います。
`control_mode` は `position` または省略です。回転 joint の位置モードと異なり、
VESC の単回転 `SET_POS` は使わず、STATUS5 の積算位置と STATUS1 の速度を使って
`VescSystem::write()` 内で PID を計算し、`SET_CURRENT` で電流を指令します。
積算位置を使うため複数回転の移動に対応し、停止位置でも PID は動作し続けます。
`position` state interface は必須で、速度状態も公開すると調整時に確認できます。

```text
目標位置 [m] → 台形 / 三角形の参照軌道 → PID → 電流上限 → VESC の電流制御
                        ↑                   ↑
                  最大速度・加速度     実測位置・実測速度

error = reference_position - measured_position
current [A] = Kp × error + integral_current + Kd × (reference_velocity - measured_velocity)
```

目標変更時も参照位置・速度を連続に保ちます。新目標が制動距離内にある場合は、
いったん減速してから戻ります。D 項には参照速度と実測速度の差を使い、
目標位置の段差をそのまま微分しません。I 項には電流単位の上限を設け、
電流が飽和する方向への積分は止めます。

前節のリード・減速比などに加え、次の設定を使います。

| 設定 | 単位・条件 |
|---|---|
| min_position / max_position | 許容位置 [m]、両方必須、URDF の lower / upper の範囲内 |
| max_velocity | 参照速度上限 [m/s]、URDF の velocity 以下。実測速度が超過しても異常停止 |
| velocity_slew_rate | 参照加速度上限 [m/s²] |
| max_current_a | 電流指令の絶対上限 [A]、正の有限値、必須 |
| position_pid_kp | P ゲイン [A/m]、正の有限値、必須 |
| position_pid_ki | I ゲイン [A/(m·s)]、0 以上、既定値 0 |
| position_pid_kd | D ゲイン [A·s/m]、0 以上、既定値 0 |
| position_pid_integral_limit_a | I 項の絶対上限 [A]、既定値 0。Ki > 0 なら正、max_current_a 以下 |
| position_pid_max_period_ms | 許容する制御周期の上限 [ms]、正の有限値、既定値 100 |

有効化時は現在位置・速度から参照軌道を初期化し、現在位置を初期目標とします。
設定した加速度で参照速度をゼロにするまでの位置が可動範囲内に入ることも確認します。
無効化・cleanup・再有効化で積分値をリセットし、前の目標は再実行しません。
範囲外・非有限の目標、実測位置の範囲外、実測速度超過、STATUS1 / STATUS5 の欠測・期限切れ、
不正・過大な制御周期は、その System の全台をゼロ電流で停止します。
これは停止位置を保持する処理ではありません。復旧には hardware lifecycle 操作が必要です。

速度・加速度制限は参照軌道に適用されます。実機の速度・加速度は質量・負荷・PID 調整に
依存し、電流指令だけで上限内の動きを保証するものではありません。実測速度の超過は
フィードバック受信後に検出するため、機械側のリミットやブレーキを置き換えません。
PID 値は機構に合わせて調整してください。設定例の値は模擬 VESC 用で、実機調整値ではありません。
タコメータの位置分解能は `linear_lead_m_per_rev / (6 × pole_pairs × gear_ratio)` [m] です。
原点復帰・絶対位置取得は未実装です。実機の位置基準を確立し、VESC 再起動後は位置基準を
取り直してから有効化してください。上位トピックの途絶検知は付属 forward controller にはありません。

```bash
ros2 launch vesc_can_ros2_control vesc_control.launch.py \
  use_mock:=true \
  urdf:=$(ros2 pkg prefix --share vesc_can_ros2_control)/examples/vesc_prismatic_position.urdf \
  controllers_file:=$(ros2 pkg prefix --share vesc_can_ros2_control)/examples/controllers_prismatic_position.yaml \
  controllers:="joint_state_broadcaster position_controller"

# 絶対位置 0.12 m を指令する（位置オフセット 0.1 m の例では +20 mm の移動）。
ros2 topic pub --once /position_controller/commands std_msgs/msg/Float64MultiArray '{data: [0.12]}'
```

## 状態の更新と配信

| 処理 | 設定例の周期 | 設定場所 |
|---|---|---|
| 各モータのCAN指令 | 100 Hz | hardware: send_period_ms=10 |
| state interface / controller | 100 Hz | controller_manager: update_rate=100 |
| JointState | 50 Hz | joint_state_broadcaster: update_rate=50 |
| 各モータのVescState | 50 Hz | hardware: state_publish_period_ms=20 |

オドメトリのcontrollerはstate interfaceを直接読めます
状態のトピックは `/<System名>_can_io/motors/<joint名>/state` です
controller_managerにnamespaceを指定した場合は、その配下に作られます
複数のSystemは名前を分け、共有CANバス上のmotor_idを重複させないようにします
例では `/VescCanSystem_can_io/motors/drive_joint/state` になります
`VescState` のlogical_idはURDFのlogical_idに対応します → [型と単位](../docs/messages.md)

state interfaceと内部計算はdoubleです
`current` と `duty_cycle` interfaceは関節方向の符号を使い、VescStateの同じ値はVESCの符号を使います

## 異常時の動作

異常時は全台にゼロ電流を送り、`/vesc/event` にlogical_idと理由を通知します
停止の保持はcontroller_managerのhardware lifecycleで復旧します
`read()` / `write()` はROSの配信を行わず、CAN用のexecutorが通信と通知を担当します

付属のforward controllerは最後の指令を保持します
実機用controllerには、上位の指令が途絶えたときのタイムアウトを設けてください
CANトピックと故障問い合わせの設定は [CANの接続](../vesc_can_driver/README.md#canの接続) を参照してください
