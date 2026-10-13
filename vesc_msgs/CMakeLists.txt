cmake_minimum_required(VERSION 3.16)
project(vesc_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(vesc_msgs
  "msg/VescState.msg"
  "msg/VescStateStamped.msg"
  DEPENDENCIES std_msgs
)

install(FILES LICENSE UPSTREAM.md DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
