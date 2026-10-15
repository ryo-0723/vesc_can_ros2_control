# vesc_can_ros2_control

複数のVESCを、1つのros2_control SystemInterfaceで管理します
controller_managerが `vesc_can_ros2_control/VescSystem` を読み込み、通常ノードは起動しません
CAN通信と状態配信には `vesc_can_driver` を使います
各Systemが通信スレッドとpublisherを所有し、cleanupで自分の通信だけを終了します

## 設定例

| URDF | コントローラのYAML | 内容 |
|---|---|---|
| [vesc_example.urdf](examples/vesc_example.urdf) | [controllers.yaml](examples/controllers.yaml) | 駆動の速度制御、操舵の位置制御 |
| [vesc_five_modes.urdf](examples/vesc_five_modes.urdf) | [controllers_five_modes.yaml](examples/controllers_five_modes.yaml) | 5モードを同じSystemで使用 |

基本例の起動手順は [ルートREADME](../README.md#ros2_controlで使う)、5モードの起動手順は [モードの説明](../docs/motor-modes.md#設定例) にあります
自分の設定は次のように指定します

```bash
ros2 launch vesc_can_ros2_control vesc_control.launch.py \
  urdf:=/path/to/robot.urdf controllers_file:=/path/to/controllers.yaml
```

## jointの設定

各jointにcommand interfaceを1つ指定します
モードは `control_mode` で選び、省略した場合はcommand interfaceから決まります
`current_velocity` は `velocity` interfaceを使います

| 設定 | 意味 |
|---|---|
| logical_id | 状態・エラーのmotor_id、0〜255で一意にする、必須 |
| controller_id | VESCのCAN ID、0〜253で一意にする、必須 |
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
指令と同名のstate interfaceも登録します
追加の状態には `position`、`velocity`、`current`、`duty_cycle`、`temperature_fet`、`temperature_motor`、`input_voltage` を指定できます

車輪のオドメトリに使う設定例です

```xml
<joint name="drive_joint">
  <param name="logical_id">1</param>
  <param name="controller_id">11</param>
  <param name="pole_pairs">7</param>
  <command_interface name="velocity"/>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
</joint>
```

位置制御以外の `position` stateは、STATUS5から求めた積算角です
車輪の移動距離は `車輪半径 × (今回の角度 − 前回の角度)` で求められます
位置制御の `position` stateは、STATUS4のPID角を関節単位へ変換します
VESCの再起動後は、オドメトリ側の前回値を取り直してください
角度の範囲と校正は [モードの説明](../docs/motor-modes.md) を参照してください

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
複数のSystemは名前を分け、共有CANバス上のcontroller_idを重複させないようにします
例では `/VescCanSystem_can_io/motors/drive_joint/state` になります
`VescState` のmotor_idはURDFのlogical_idに対応します → [型と単位](../docs/messages.md)

state interfaceと内部計算はdoubleです
`current` と `duty_cycle` interfaceは関節方向の符号を使い、VescStateの同じ値はVESCの符号を使います

## 異常時の動作

異常時は全台にゼロ電流を送り、`/vesc/event` にmotor_idと理由を通知します
停止の保持はcontroller_managerのhardware lifecycleで復旧します
`read()` / `write()` はROSの配信を行わず、CAN用のexecutorが通信と通知を担当します

付属のforward controllerは最後の指令を保持します
実機用controllerには、上位の指令が途絶えたときのタイムアウトを設けてください
CANトピックと故障問い合わせの設定は [CANの接続](../vesc_can_driver/README.md#canの接続) を参照してください
