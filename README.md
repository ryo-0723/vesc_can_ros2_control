# vesc_can_ros2_control

ROS 2 driver for **native VESC CAN frames**, with a multi-joint `ros2_control`
SystemInterface and a standalone topic node. Intended for CAN bridges, including
Zenoh/pico gateways that preserve `can_msgs/msg/Frame` IDs, flags, DLC and payload.
It does not implement USB/UART packet transport or a Zenoh/pico gateway.

Initial supported build target: **ROS 2 Lyrical / ros2_control 6**. Other distributions
have different hardware APIs and have not been validated. The C++ library explicitly
requires C++20, matching the Lyrical dependencies. This is a development
package; no ROS apt release or physical-hardware validation is claimed.

Local validation (2026-10-15): Lyrical build and all 70 test cases passed with
`rmw_fastrtps_cpp`, including standalone and ros2_control integration tests.
Installed C++ library and independent command/state/event message usage also passed.
See the [validation report](docs/validation.md). The physical CAN bus
and Zenoh pico gateway have not been exercised.

Motor modes: `current` (motor A signed in the joint direction), `velocity` (joint rad/s),
`position` (joint rad), `duty_cycle` (signed duty ratio), and
`current_velocity` (current startup followed by velocity control).
Both standalone and ros2_control use the same driver functions for all five modes.
Select each motor\'s mode at startup; standalone uses YAML `control_mode`.

Default rates: CAN commands are sent at 100 Hz per motor; ros2_control state interfaces
update at 100 Hz. Standalone publishes all motors on `/vesc/state` as VescStateArray
at 50 Hz; ros2_control publishes per-motor VescState at 50 Hz.
JointState publishes at 50 Hz through ros2_control's joint_state_broadcaster.
The standalone node publishes periodic state arrays and error events.
Fresh commands start control automatically; `~/reboot` takes a logical motor ID and
sends COMM_REBOOT once to that VESC. Stored commands are cleared before recovery.
VESC fault codes are queried over CAN at 10 Hz; both frontends publish detected faults
immediately on `/vesc/event` using `vesc_can_msgs/msg/VescEvent` (logical motor_id and error text).
See the [rate settings](vesc_can_ros2_control/README.md#更新と状態配信の周期).

- [CAN protocol, standalone node and bridge configuration (日本語)](vesc_can_driver/README.md)
- [ros2_control plugin and controller examples (日本語)](vesc_can_ros2_control/README.md)
- [Architecture and concurrency](docs/architecture.md)
- [Source layout and file responsibilities (日本語)](docs/source-layout.md)
- [Motor control modes, units and stop behavior (日本語)](docs/motor-modes.md)
- [Release preparation](docs/releasing.md)
- All project-owned messages and services are defined in `vesc_can_msgs`:
  VescCommand, timestamped VescCommandArray, VescState, VescStateArray, VescEvent and Reboot.
  VescState contains converted joint velocity (`velocity_rad_s`, rad/s), a STATUS4-derived
  PID joint angle (`pid_pos_rad`, rad) and a STATUS5-derived accumulated joint angle
  (`position_rad`, rad). Motion feedback uses float64; monitoring values use float32.
  Missing or expired values are NaN. State and events use the same logical motor_id as commands.
  Configure logical_id in standalone YAML and every ros2_control joint's URDF parameters.
  Standalone command values use float32.
  ros2_control joint state uses standard ROS messages. The CAN boundary uses `can_msgs/msg/Frame`.
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
colcon test --packages-select vesc_can_driver vesc_can_ros2_control
colcon test-result --verbose
```

The repository contains three ROS packages:

| Package | Purpose |
|---|---|
| `vesc_can_msgs` | Independent command, state, error and reboot interfaces; no driver dependency |
| `vesc_can_driver` | Native CAN encode/decode, common driver and transport, standalone node and fake VESC |
| `vesc_can_ros2_control` | Multi-joint hardware plugin, control launch and URDF/controller examples; depends on `vesc_can_driver` |

To use the CAN codec or standalone node without ros2_control, build only its
dependency chain:

```bash
rosdep install --from-paths src/vesc_can_ros2_control/vesc_can_driver \
  src/vesc_can_ros2_control/vesc_can_msgs \
  --ignore-src -r -y --rosdistro lyrical
colcon build --packages-up-to vesc_can_driver --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select vesc_can_driver
```

The reusable codec target is `vesc_can_driver::protocol`; see its
[C++ usage example](vesc_can_driver/README.md#ライブラリとして使う).
No extra VESC repository checkout is needed.
Command producers and state subscribers can depend only on `vesc_can_msgs`; see its
[C++ and Python usage](vesc_can_msgs/README.md).
The state schema is independently defined for this CAN driver. It uses a direct
`stamp` field and flat values; update subscribers that previously used
`vesc_msgs/msg/VescStateStamped`. See the [message mapping](docs/messages.md).

## Code style

C++ uses the repository's `.clang-format`; Python uses Black with a 100-character
line limit. CI checks both before building:

```bash
clang-format -i vesc_can_ros2_control/include/vesc_can_ros2_control/*.hpp \
  vesc_can_ros2_control/src/*.cpp vesc_can_ros2_control/test/*.cpp
clang-format -i vesc_can_driver/include/vesc_can_driver/*.hpp vesc_can_driver/src/*.cpp \
  vesc_can_driver/test/*.cpp vesc_can_driver/test/support/*.cpp
black --line-length 100 vesc_can_driver/launch vesc_can_ros2_control/launch \
  vesc_can_ros2_control/test/test_integration.py
```

The standalone API uses VescCommandArray commands, VescStateArray states and error events; see the
[migration guide](docs/messages.md). C++ users must update the renamed identifiers:
`ControlMode::Current`, `ControlMode::Velocity`, `ControlMode::Position`,
`ControlMode::DutyCycle`, and `ControlMode::CurrentVelocity`, `kPi`, `kUnknownValue`, the protocol's `k*Id` constants,
and `DriverCore::motor_configs()`. PC-side `enable/disable/enabled` have been renamed
to `activate/deactivate/active`; the standalone activation service is replaced by `~/reboot`.
Logical IDs are required in standalone YAML and hardware URDF; state and events now expose motor_id.
Replace erpm with velocity_rad_s, pid_pos_deg with pid_pos_rad and raw tachometer fields with position_rad.
The error topic type is VescEvent. Rebuild interface consumers together.
`MotorConfig` includes motor_id alongside controller_id, and
`CanTransport::set_commands()` accepts a batch of MotorCommand (motor index and value).
Unsent values are overwritten per motor; one batch uses one lock and receipt time.
The driver implementation has four files: `vesc_protocol`, `vesc_driver`, `vesc_node`, and
`vesc_state`. Include `vesc_driver.hpp` for shared motor types, control and CAN transport,
and `vesc_state.hpp` for VescState conversion/publication. The shared targets are
`vesc_can_driver::driver` and `vesc_can_driver::state`; the codec target remains `::protocol`.
Both standalone and ros2_control use the same driver and state libraries.
Shared C++ headers now use `vesc_can_driver/...` and namespace `vesc_can_driver`.
The CAN package name and CMake target prefix are also `vesc_can_driver`
(for example, `vesc_can_driver::protocol`).
The standalone launch command is now `ros2 launch vesc_can_driver vesc_standalone.launch.py`.

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
STATUS decoding is pinned to firmware 6.06, commit
`94b305ec124b67b0161f2d3d85c4d5b35e48edc5`; see the
[native STATUS layout](docs/messages.md#vesc-firmware-6-の-status-形式).
No VESC firmware source or sbgisen implementation is bundled.

The command, state, error and reboot interfaces are independently defined in `vesc_can_msgs`.
The copied F1TENTH state package has been removed; its definitions are not used by this driver.
The root [LICENSE](LICENSE) is the MIT license for all three packages.
Each package installs this original into its share directory.
Keep the repository layout when building; their CMake install rules refer to `../LICENSE`.
