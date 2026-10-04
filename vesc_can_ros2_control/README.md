# vesc_can_ros2_control

VESCのRPM・電流指令をCANフレームへ変換し、STATUS1から回転速度と電流を取得するROS 2パッケージです。
複数のVESCを1つの`vesc_node`で管理します。

## 名前と構成

- パッケージ名・C++名前空間: `vesc_can_ros2_control`
- 実行ファイル: `vesc_node`
- 既定のROSノード名: `vesc_can_ros2_control`
- ヘッダー: `include/vesc_can_ros2_control/vesc_protocol.hpp`
- メッセージ依存: `vesc_can_interfaces`

## ROSインターフェース

| 種別 | 既定トピック | 型 |
|---|---|---|
| subscribe | `/vesc/target` | `vesc_can_interfaces/msg/ActuatorTarget` |
| subscribe | `/vesc/target_array` | `vesc_can_interfaces/msg/ActuatorTargetArray` |
| publish | `/vesc/state` | `vesc_can_interfaces/msg/ActuatorState` |
| publish | `/vesc/state_array` | `vesc_can_interfaces/msg/ActuatorStateArray` |
| publish | `/socketcan_bridge/tx` | `can_msgs/msg/Frame` |
| subscribe | `/socketcan_bridge/rx` | `can_msgs/msg/Frame` |

指令と状態の速度単位は機械RPMです。CAN送受信トピックは`can_tx_topic`と
`can_rx_topic`で変更でき、Zenoh経由の中継先にも接続できます。

## 起動と移行

```bash
ros2 run vesc_can_ros2_control vesc_node --ros-args --params-file <モーター設定.yaml>
```

パラメーターファイルで旧ノード名`/vesc_driver`を指定している場合は、
`/vesc_can_ros2_control`へ変更してください。launchでノード名を指定する場合は、
その名前とパラメーターファイルのキーを合わせてください。
旧`actuator_msgs`を使用する外部ノードは、依存先・型名を`vesc_can_interfaces`へ変更する必要があります。

今回の整理ではCAN形式、トピック名、メッセージのフィールド、制御ロジックは変更していません。
ros2_controlのハードウェアプラグインは未実装です。
