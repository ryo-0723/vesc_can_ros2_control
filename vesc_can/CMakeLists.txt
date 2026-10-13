cmake_minimum_required(VERSION 3.16)
project(vesc_can)
set(CMAKE_CXX_EXTENSIONS OFF)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(can_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(vesc_msgs REQUIRED)
find_package(Threads REQUIRED)

add_library(vesc_can_protocol SHARED src/vesc_protocol.cpp)
add_library(vesc_can::protocol ALIAS vesc_can_protocol)
set_target_properties(vesc_can_protocol PROPERTIES EXPORT_NAME protocol)
target_compile_features(vesc_can_protocol PUBLIC cxx_std_20)
target_include_directories(vesc_can_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(vesc_can_protocol PUBLIC ${can_msgs_TARGETS})

add_library(vesc_can_core SHARED src/driver_core.cpp)
add_library(vesc_can::core ALIAS vesc_can_core)
set_target_properties(vesc_can_core PROPERTIES EXPORT_NAME core)
target_link_libraries(vesc_can_core PUBLIC vesc_can::protocol)

add_library(vesc_can_transport SHARED src/can_transport.cpp)
add_library(vesc_can::transport ALIAS vesc_can_transport)
set_target_properties(vesc_can_transport PROPERTIES EXPORT_NAME transport)
target_link_libraries(vesc_can_transport PUBLIC vesc_can::core rclcpp::rclcpp Threads::Threads)

add_library(vesc_can_state_conversion SHARED src/vesc_state_conversion.cpp)
add_library(vesc_can::state_conversion ALIAS vesc_can_state_conversion)
set_target_properties(vesc_can_state_conversion PROPERTIES EXPORT_NAME state_conversion)
target_link_libraries(vesc_can_state_conversion PUBLIC vesc_can::core ${vesc_msgs_TARGETS})

add_executable(vesc_node src/vesc_node.cpp)
target_link_libraries(vesc_node PRIVATE vesc_can::transport vesc_can::state_conversion
  ${sensor_msgs_TARGETS} ${diagnostic_msgs_TARGETS} ${std_msgs_TARGETS} ${std_srvs_TARGETS})
# Installed even with BUILD_TESTING=OFF so use_mock launch demos remain available.
add_executable(fake_vesc_node test/support/fake_vesc_node.cpp)
target_link_libraries(fake_vesc_node PRIVATE vesc_can::core rclcpp::rclcpp)

install(TARGETS vesc_can_protocol vesc_can_core vesc_can_transport vesc_can_state_conversion
  EXPORT export_vesc_can
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS vesc_node fake_vesc_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md "${CMAKE_CURRENT_SOURCE_DIR}/../LICENSE"
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol_core test/test_protocol_core.cpp)
  target_link_libraries(test_protocol_core vesc_can::core)
  ament_add_gtest(test_transport test/test_transport.cpp TIMEOUT 30)
  target_link_libraries(test_transport vesc_can::transport)
  ament_add_gtest(test_state_conversion test/test_state_conversion.cpp)
  target_link_libraries(test_state_conversion vesc_can::state_conversion)
endif()

ament_export_targets(export_vesc_can HAS_LIBRARY_TARGET NAMESPACE vesc_can::)
ament_export_dependencies(can_msgs rclcpp vesc_msgs Threads)
ament_package()
