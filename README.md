# vesc_can_ros2_control

ROS 2 driver for **native VESC CAN frames**, with a multi-joint `ros2_control`
SystemInterface and a standalone topic node. Intended for CAN bridges, including
Zenoh/pico gateways that preserve `can_msgs/msg/Frame` IDs, flags, DLC and payload.
It does not implement USB/UART packet transport or a Zenoh/pico gateway.

Initial supported build target: **ROS 2 Lyrical / ros2_control 6**. Other distributions
have different hardware APIs and have not been validated. The C++ library explicitly
requires C++20, matching the Lyrical dependencies. This is a development
package; no ROS apt release or physical-hardware validation is claimed.

Local validation (2026-10-14): Lyrical build and all 40 test cases passed with
`rmw_fastrtps_cpp`, including standalone and ros2_control integration tests.
Installed codec usage and a `BUILD_TESTING=OFF` build also passed.
See the [validation report](docs/validation.md). The physical CAN bus
and Zenoh pico gateway have not been exercised.

Motor modes: `current` (motor A signed in the joint direction), `velocity` (joint rad/s),
`position` (joint rad), and `current_velocity` (current startup followed by velocity control).
Both standalone and ros2_control use the same driver functions for all four modes.

Default rates: CAN commands are sent at 100 Hz per motor; ros2_control state interfaces
update at 100 Hz. JointState and per-motor VescStateStamped topics publish at 50 Hz
in both frontends. See the [rate settings](vesc_can_ros2_control/README.md#更新と状態配信の周期).

- [CAN protocol, standalone node and bridge configuration (日本語)](vesc_can_driver/README.md)
- [ros2_control plugin and controller examples (日本語)](vesc_can_ros2_control/README.md)
- [Architecture and concurrency](docs/architecture.md)
- [Source layout and file responsibilities (日本語)](docs/source-layout.md)
- [Motor control modes, units and stop behavior (日本語)](docs/motor-modes.md)
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
colcon test --packages-select vesc_can_driver vesc_can_ros2_control
colcon test-result --verbose
```

The repository contains three ROS packages:

| Package | Purpose |
|---|---|
| `vesc_can_driver` | Native CAN encode/decode, common driver and transport, standalone node and fake VESC |
| `vesc_can_ros2_control` | Multi-joint hardware plugin, control launch and URDF/controller examples; depends on `vesc_can_driver` |
| `vesc_msgs` | State-only subset: two message definitions copied from a pinned f1tenth ROS 2 revision |

To use the CAN codec or standalone node without ros2_control, build only its
dependency chain:

```bash
rosdep install --from-paths src/vesc_can_ros2_control/vesc_can_driver \
  src/vesc_can_ros2_control/vesc_msgs --ignore-src -r -y --rosdistro lyrical
colcon build --packages-up-to vesc_can_driver --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select vesc_can_driver
```

The reusable codec target is `vesc_can_driver::protocol`; see its
[C++ usage example](vesc_can_driver/README.md#ライブラリとして使う).
No extra VESC repository checkout is needed.
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
clang-format -i vesc_can_driver/include/vesc_can_driver/*.hpp vesc_can_driver/src/*.cpp \
  vesc_can_driver/test/*.cpp vesc_can_driver/test/support/*.cpp
black --line-length 100 vesc_can_driver/launch vesc_can_ros2_control/launch \
  vesc_can_ros2_control/test/test_integration.py
```

The standalone API now uses upstream and standard messages; see the
[migration guide](docs/messages.md). C++ users must update the renamed identifiers:
`ControlMode::Current`, `ControlMode::Velocity`, `ControlMode::Position`,
and `ControlMode::CurrentVelocity`, `kPi`, `kUnknownValue`, the protocol's `k*Id` constants,
and `DriverCore::motor_configs()`. `MotorConfig::logical_id` is removed, and
`CanTransport::set_command()` takes a motor index.
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

The `vesc_msgs` package is copied from
[f1tenth/vesc](https://github.com/f1tenth/vesc/tree/153998df8545fe1781b975df88e411b4e71d4bfe/vesc_msgs).
Only `VescState.msg` and `VescStateStamped.msg` are copied, with their contents
unchanged. IMU messages are excluded. CMake and package metadata are maintained
locally; this is a state-only subset of the upstream package.
The upstream BSD-3-Clause license is preserved in [vesc_msgs/LICENSE](vesc_msgs/LICENSE);
the repository's MIT license applies to the driver code.
The message package also installs its license and provenance into `share/vesc_msgs`.
The source revision and local additions are recorded in
[vesc_msgs/UPSTREAM.md](vesc_msgs/UPSTREAM.md).

License sources are limited to the root [LICENSE](LICENSE) (MIT for the driver)
and [vesc_msgs/LICENSE](vesc_msgs/LICENSE) (BSD-3-Clause for the copied messages).
Both driver packages install the root MIT license into their own share directories.
Keep the repository layout when building; their CMake install rules refer to `../LICENSE`.
See [message dependency choices](docs/messages.md#コピーと-vcs-取得の使い分け)
for the choice between vendoring and `vcs import`.
