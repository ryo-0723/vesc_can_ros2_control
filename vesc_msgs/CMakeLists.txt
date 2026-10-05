cmake_minimum_required(VERSION 3.5)
project(vesc_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(vesc_msgs
  "msg/VescState.msg"
  "msg/VescStateStamped.msg"
  "msg/VescImu.msg"
  "msg/VescImuStamped.msg"
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(FILES LICENSE UPSTREAM.md DESTINATION share/${PROJECT_NAME})

ament_auto_package()
