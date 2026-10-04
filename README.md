# vesc_can_ros2_control

VESCのCANフレーム生成・受信処理を土台として、ros2_control対応を開発するリポジトリです。
現在は通常のROS 2ノードによるRPM・電流指令とSTATUS1の受信に対応しています。
ros2_controlのハードウェアプラグインは今後追加します。

## パッケージ

- [vesc_can_ros2_control](vesc_can_ros2_control/README.md): VESC用CANプロトコルと`vesc_node`。
- `vesc_can_interfaces`: 既存ノードの指令・状態メッセージとサービス定義。

独自インターフェースは、既存のROSパッケージとの名前の重複を避けるため、
`actuator_msgs`から`vesc_can_interfaces`へ変更しています。定義内容は同じです。

## コードの出典

初期のVESC実装は[rox2026のhardware_driver](https://github.com/rodep-soft/rox2026/tree/main/ros2_ws/src/hardware_driver)を基にしています。
