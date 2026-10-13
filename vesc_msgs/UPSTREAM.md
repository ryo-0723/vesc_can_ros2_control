# Upstream provenance

This directory contains two unchanged ROS 2 message definitions from
[f1tenth/vesc](https://github.com/f1tenth/vesc), branch `ros2`, at commit
`153998df8545fe1781b975df88e411b4e71d4bfe` (2023-03-27).

[Browse the exact source revision](https://github.com/f1tenth/vesc/tree/153998df8545fe1781b975df88e411b4e71d4bfe/vesc_msgs).

Only `msg/VescState.msg` and `msg/VescStateStamped.msg` are copied. Their contents,
field types, constants, and units are unchanged. `VescImu.msg` and
`VescImuStamped.msg` are excluded, as this CAN driver does not publish IMU data.
No upstream driver, node, launch file, or changelog is bundled.

The package name remains `vesc_msgs` so existing state publishers and subscribers
keep their interface names. This is a **state-only subset**, not a complete copy
of the upstream package. Applications requiring its IMU interfaces must use the
full upstream package instead. Do not install two source packages named
`vesc_msgs` in one workspace.

`CMakeLists.txt` and `package.xml` are maintained locally as the minimal ROS 2
build wrapper for those two messages. The local package version is `0.1.0`;
the copied upstream package was version `1.1.0`. The wrapper uses `ament_cmake`,
`rosidl_default_generators`, `std_msgs`, and `rosidl_default_runtime`; it does not
depend on `geometry_msgs`. `builtin_interfaces` is supplied through `std_msgs`.
The local maintainer and repository URLs identify this repository. The original
authors Michael T. Boulet and Joshua Whitley remain in `package.xml`.
The upstream maintainer at the pinned revision was Johannes Betz
(`joebetz@seas.upenn.edu`).

`LICENSE` preserves the upstream BSD-3-Clause text and the F1TENTH Foundation
copyright notice. This license and provenance record are installed into
`share/vesc_msgs`. The repository's root MIT license applies to the driver code;
it does not replace this BSD license.

Message checksums (SHA-256):

```text
07ac060cd7fa48be578e87f70132c9048c7af5df743f258ddb2af29e31fefc33  msg/VescState.msg
d4a669c957bb7a7f1c2bc8b19db646324af5e445368b3962ebd86b3a7778665e  msg/VescStateStamped.msg
```

When updating these definitions, record the new upstream commit and checksums,
and validate the driver's message mapping and integration tests.

Before an apt release, coordinate the `vesc_msgs` release source and maintenance
with upstream. Release this package alongside the driver only if that release
arrangement is agreed; otherwise use the upstream release for the target ROS
distribution. This subset has not been released independently to a ROS
distribution. Do not register a second provider for the same package in one
ROS distribution or present this subset as the complete upstream package.
