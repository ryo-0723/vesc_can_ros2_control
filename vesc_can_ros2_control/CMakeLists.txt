cmake_minimum_required(VERSION 3.16)
project(vesc_can_ros2_control)
set(CMAKE_CXX_EXTENSIONS OFF)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(vesc_can_driver REQUIRED)

add_library(vesc_hardware SHARED src/vesc_system.cpp)
target_include_directories(vesc_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(vesc_hardware PUBLIC vesc_can_driver::driver vesc_can_driver::state
  hardware_interface::hardware_interface pluginlib::pluginlib rclcpp_lifecycle::rclcpp_lifecycle)
pluginlib_export_plugin_description_file(hardware_interface vesc_plugins.xml)

install(TARGETS vesc_hardware EXPORT export_vesc_hardware
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY examples launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md "${CMAKE_CURRENT_SOURCE_DIR}/../LICENSE"
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_system test/test_system.cpp)
  target_link_libraries(test_system vesc_hardware)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_integration.py TIMEOUT 60)
endif()

ament_export_targets(export_vesc_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(vesc_can_driver hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
