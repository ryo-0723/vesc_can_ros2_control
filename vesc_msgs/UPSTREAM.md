# Upstream provenance

This directory contains the ROS 2 `vesc_msgs` package from
[f1tenth/vesc](https://github.com/f1tenth/vesc), branch `ros2`, at commit
`153998df8545fe1781b975df88e411b4e71d4bfe` (2023-03-27).

[Browse the exact source revision](https://github.com/f1tenth/vesc/tree/153998df8545fe1781b975df88e411b4e71d4bfe/vesc_msgs).

The four `.msg` files, `package.xml`, and `CHANGELOG.rst` are copied without
changes. The package name remains `vesc_msgs`, preserving the
ROS interface names, field types, constants, and units of this revision.

Local additions:

- `LICENSE`: an unchanged copy of the upstream repository's BSD-3-Clause license,
  including the F1TENTH Foundation copyright notice.
- `UPSTREAM.md`: this provenance record.
- `CMakeLists.txt`: one install rule to include `LICENSE` and `UPSTREAM.md` in
  `share/vesc_msgs` when installing the generated interfaces. The upstream
  interface generation and dependency configuration are unchanged.

The root MIT license of `vesc_can_ros2_control` does not replace the BSD-3-Clause
license of this package. The original author and maintainer information in
`package.xml` is retained for attribution; this development copy has not been
released independently to a ROS distribution.

Build this package from the same workspace as the CAN driver. Do not import a
second source package named `vesc_msgs` into that workspace. If updating the
vendored package, record the new upstream commit here and verify all message
definitions against that revision before changing the driver.

Before an apt release, coordinate the `vesc_msgs` release source and maintenance
with upstream. Release this package alongside the driver only if that release
arrangement is agreed; otherwise use the upstream release for the target ROS
distribution. Do not register a second provider for the same package in one
ROS distribution.
