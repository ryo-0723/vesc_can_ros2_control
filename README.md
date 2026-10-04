# vesc_can_ros2_control

ROS 2 driver for **native VESC CAN frames**, with a multi-joint `ros2_control`
SystemInterface and a standalone topic node. Intended for CAN bridges, including
Zenoh/pico gateways that preserve `can_msgs/msg/Frame` IDs, flags, DLC and payload.
It does not implement USB/UART packet transport or a Zenoh/pico gateway.

Initial supported build target: **ROS 2 Lyrical / ros2_control 6**. Other distributions
have different hardware APIs and have not been validated. This is a development
package; no ROS apt release or physical-hardware validation is claimed.

Local validation: Lyrical build and 17 automated checks passed. Driver and integration
tests also passed with `rmw_zenoh_cpp` and a local Zenoh router. The physical CAN bus
and Zenoh pico gateway have not been exercised.

- [Driver, configuration, examples and limitations (日本語)](vesc_can_ros2_control/README.md)
- [Architecture and concurrency](docs/architecture.md)
- [Release preparation](docs/releasing.md)
- `vesc_can_interfaces`: retained legacy messages; the hardware plugin uses standard
  ros2_control interfaces, and the CAN boundary uses `can_msgs/msg/Frame`.

## Build and test

Place this repository under a ROS workspace's `src` directory, then run from the
workspace root:

```bash
source /opt/ros/lyrical/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro lyrical
colcon build --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test
colcon test-result --verbose
```

## Bench demo without a motor

Use isolated CAN topics when physical bridges are running:

```bash
ros2 launch vesc_can_ros2_control vesc_control.launch.py use_mock:=true \
  can_tx_topic:=/bench/can/tx can_rx_topic:=/bench/can/rx
ros2 topic pub -r 20 /drive_controller/commands std_msgs/msg/Float64MultiArray '{data: [1.0]}'
ros2 topic pub -r 20 /steer_controller/commands std_msgs/msg/Float64MultiArray '{data: [0.25]}'
```

The example forwarding controllers retain their last commands. They are for bench
verification and do not provide an upstream command timeout. Use a controller or
supervisor with a watchdog for the actual robot.

## Provenance

The original standalone node and legacy interfaces were based on
[rox2026/hardware_driver](https://github.com/rodep-soft/rox2026/tree/main/ros2_ws/src/hardware_driver).
Their initial package names were `hardware_driver` and `actuator_msgs`.
Native frame layouts were checked against
[VESC firmware comm_can.c](https://github.com/vedderb/bldc/blob/master/comm/comm_can.c).
No VESC firmware source or sbgisen implementation is bundled.
