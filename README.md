# vesc_can_ros2_control

ROS 2 driver for **native VESC CAN frames**, with a multi-joint `ros2_control`
SystemInterface and a standalone topic node. Intended for CAN bridges, including
Zenoh/pico gateways that preserve `can_msgs/msg/Frame` IDs, flags, DLC and payload.
It does not implement USB/UART packet transport or a Zenoh/pico gateway.

Initial supported build target: **ROS 2 Lyrical / ros2_control 6**. Other distributions
have different hardware APIs and have not been validated. The C++ library explicitly
requires C++20, matching the Lyrical dependencies. This is a development
package; no ROS apt release or physical-hardware validation is claimed.

Local validation: Lyrical build and 20 automated checks passed. Driver and integration
tests also passed with `rmw_zenoh_cpp` and a local Zenoh router. The physical CAN bus
and Zenoh pico gateway have not been exercised.

- [Driver, configuration, examples and limitations (日本語)](vesc_can_ros2_control/README.md)
- [Architecture and concurrency](docs/architecture.md)
- [Release preparation](docs/releasing.md)
- State telemetry uses the bundled upstream `vesc_msgs/msg/VescStateStamped`; joint commands
  and state use standard ROS messages. The CAN boundary uses `can_msgs/msg/Frame`.
- [Message mapping and migration (日本語)](docs/messages.md)

## Build and test

Place this repository under a ROS workspace's `src` directory, then run from the
workspace root:

```bash
source /opt/ros/lyrical/setup.bash
rosdep install --from-paths src/vesc_can_ros2_control \
  --ignore-src -r -y --rosdistro lyrical
colcon build --packages-up-to vesc_can_ros2_control --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select vesc_can_ros2_control
colcon test-result --verbose
```

The repository contains two ROS packages: the CAN driver and `vesc_msgs`, copied
from a pinned f1tenth ROS 2 revision. No extra VESC repository checkout is needed.
Do not add a second source copy of `vesc_msgs` to the same workspace. Existing
workspaces using the previous `dependencies.repos` should move the old
`src/vesc_upstream` checkout outside `src` before rebuilding.
See [upstream provenance](vesc_msgs/UPSTREAM.md) and
[release preparation](docs/releasing.md) for the apt release policy.

## Code style

C++ uses the repository's `.clang-format`; Python uses Black with a 100-character
line limit. CI checks both before building:

```bash
clang-format -i vesc_can_ros2_control/include/vesc_can_ros2_control/*.hpp \
  vesc_can_ros2_control/src/*.cpp vesc_can_ros2_control/test/*.cpp
black --line-length 100 vesc_can_ros2_control/launch \
  vesc_can_ros2_control/test/test_integration.py
```

The standalone API now uses upstream and standard messages; see the
[migration guide](docs/messages.md). C++ users must update the renamed identifiers:
`ControlMode::Velocity`
and `ControlMode::Position`, `kPi`, `kUnknownValue`, the protocol's `k*Id` constants,
and `DriverCore::motor_configs()`. `MotorConfig::logical_id` is removed, and
`CanTransport::set_command()` takes a motor index.

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

The `vesc_msgs` package is copied from
[f1tenth/vesc](https://github.com/f1tenth/vesc/tree/153998df8545fe1781b975df88e411b4e71d4bfe/vesc_msgs).
Its four message definitions and original package metadata are unchanged.
The upstream BSD-3-Clause license is preserved in [vesc_msgs/LICENSE](vesc_msgs/LICENSE);
the repository's MIT license applies to the driver code.
The message package also installs its license and provenance into `share/vesc_msgs`.
The source revision and local additions are recorded in
[vesc_msgs/UPSTREAM.md](vesc_msgs/UPSTREAM.md).
