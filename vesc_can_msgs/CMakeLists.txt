cmake_minimum_required(VERSION 3.16)
project(vesc_can_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VescCommand.msg"
  "msg/VescCommandArray.msg"
  "msg/VescState.msg"
  "srv/Reboot.srv"
  DEPENDENCIES builtin_interfaces
)

install(FILES README.md "${CMAKE_CURRENT_SOURCE_DIR}/../LICENSE"
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime builtin_interfaces)
ament_package()
