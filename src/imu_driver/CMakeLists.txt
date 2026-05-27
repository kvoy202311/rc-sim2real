cmake_minimum_required(VERSION 3.8)
project(imu_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kvoy_msgs REQUIRED)

add_executable(imu_driver_node
  src/imu_driver_node.cpp
  src/serial_port.cpp
  src/wit_motion_imu_node.cpp
)

target_include_directories(imu_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(imu_driver_node rclcpp kvoy_msgs)

install(TARGETS imu_driver_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/imu_diagnostics.py DESTINATION lib/${PROJECT_NAME})

ament_package()
