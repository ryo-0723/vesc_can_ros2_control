# vesc_can_ros2_control

VESCをCANで制御するROS 2パッケージです
通常ノードで指令と状態をトピックに流す方法と、ros2_controlのhardware componentとして使う方法があります
対応環境は **ROS 2 Lyrical / ros2_control 6**、C++20です

## 構成

| フォルダ | 内容 |
|---|---|
| [vesc_can_msgs](vesc_can_msgs/README.md) | 指令・状態・エラーのmsg、再起動のsrv |
| [vesc_can_driver](vesc_can_driver/README.md) | CANプロトコル、通信管理、通常ノード |
| [vesc_can_ros2_control](vesc_can_ros2_control/README.md) | ros2_controlのhardware plugin |
| [docs](docs/) | モード、メッセージ、内部構成、検証の説明 |

YAMLとURDFの設定例は、各パッケージの `examples/` にまとめています
`src/` は実装、`include/` は公開ヘッダ、`launch/` は起動、`test/` はテストです

## ビルド

リポジトリをワークスペースの `src/` に置き、ワークスペースのルートで実行します

```bash
source /opt/ros/lyrical/setup.bash
rosdep install --from-paths src/vesc_can_ros2_control --ignore-src -r -y --rosdistro lyrical
colcon build --packages-up-to vesc_can_ros2_control --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

通常ノードだけを使う場合は、ビルド対象を `--packages-up-to vesc_can_driver` にします
上位ノードでmsgだけを使う場合は、`vesc_can_msgs` に依存すれば十分です

## 通常ノードで使う

[standalone.yaml](vesc_can_driver/examples/standalone.yaml) は、速度制御の駆動モータと位置制御の操舵モータの例です
まず模擬VESCで起動できます

```bash
ros2 launch vesc_can_driver vesc_standalone.launch.py use_mock:=true
```

別の端末で、2台の指令を周期的に送ります

```bash
ros2 topic pub -r 20 /vesc/commands vesc_can_msgs/msg/VescCommandArray \
  '{stamp: now, commands: [{motor_id: 1, value: 1.0}, {motor_id: 2, value: 0.25}]}'
ros2 topic echo /vesc/state
```

実機ではCANブリッジを起動し、`use_mock` を省略します
CANの送受信には `can_msgs/msg/Frame` を使います
模擬VESCと実機のブリッジを併用するときは、CANトピックを分けてください
自分のYAMLは `config:=/path/to/motors.yaml` で指定できます → [設定の説明](vesc_can_driver/README.md)

## ros2_controlで使う

こちらを使うときは、通常ノードを起動しません
URDFの各jointに、論理IDの `logical_id` とCAN IDの `controller_id` を指定します

```bash
ros2 launch vesc_can_ros2_control vesc_control.launch.py use_mock:=true \
  can_tx_topic:=/bench/can/tx can_rx_topic:=/bench/can/rx
```

別の端末で、コントローラへ指令を送ります

```bash
ros2 topic pub -r 20 /drive_controller/commands std_msgs/msg/Float64MultiArray '{data: [1.0]}'
ros2 topic pub -r 20 /steer_controller/commands std_msgs/msg/Float64MultiArray '{data: [0.25]}'
```

実機のURDFとコントローラ設定は、`urdf` と `controllers_file` で指定します → [設定の説明](vesc_can_ros2_control/README.md)

## モード

モードはモータごとに起動時の設定で決めます

| control_mode | 指令値 | 動作 |
|---|---|---|
| velocity | 関節速度 [rad/s] | 速度制御 |
| current | モータ電流 [A] | 電流制御 |
| position | 関節角度 [rad] | 位置制御 |
| duty_cycle | duty比 [-1, 1] | duty比を指定 |
| current_velocity | 関節速度 [rad/s] | 電流で始動し、速度の閾値に達したら速度制御 |

減速比、方向、指令の上限、停止時の動作は [モードの説明](docs/motor-modes.md) を参照してください

## 状態とエラー

通常ノードの `/vesc/state` は全モータをまとめた `VescStateArray` です
指令・状態・エラーは、同じ論理 `motor_id` で識別します
速度は `velocity_rad_s`、積算角は `position_rad`、PID角は `pid_pos_rad` です
速度・角度はfloat64、電流・温度・電圧などの監視値はfloat32で、欠測値はNaNにします

| 処理 | 既定の周期 |
|---|---|
| 各モータのCAN指令 | 100 Hz |
| VescState / VescStateArray | 50 Hz |
| ros2_controlのstate interface | 100 Hz |
| joint_state_broadcasterのJointState | 50 Hz |

異常は `/vesc/event` の `VescEvent`（motor_id、message）に通知します
VESCの故障コードは100 msごとに問い合わせ、応答を受けた処理から通知します
異常時は全台にゼロ電流を送り、停止を保持します
通常ノードは `~/reboot`、ros2_controlはhardware lifecycleの復旧操作を使います → [型と単位の説明](docs/messages.md)

## テスト

```bash
colcon test --packages-select vesc_can_driver vesc_can_ros2_control
colcon test-result --verbose
```

模擬CANで70ケースを確認しています
実VESC・実CANバス・Zenoh picoの中継器は未検証です → [検証記録](docs/validation.md)

## 参考・ライセンス

初期の通常ノードは [rox2026/hardware_driver](https://github.com/rodep-soft/rox2026/tree/main/ros2_ws/src/hardware_driver) を参考にしています
CANの形式は [公式VESC firmware 6.06](https://github.com/vedderb/bldc/tree/94b305ec124b67b0161f2d3d85c4d5b35e48edc5) に合わせています
msgとsrvは独自定義です
ライセンスは3パッケージ共通の [MIT](LICENSE) です
