cmake_minimum_required(VERSION 3.8)
project(serial_comm)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kvoy_msgs REQUIRED)

add_executable(serial_comm_node
  src/serial_comm_node.cpp
  src/serial_port.cpp
)

target_include_directories(serial_comm_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(serial_comm_node rclcpp kvoy_msgs)

install(TARGETS serial_comm_node DESTINATION lib/${PROJECT_NAME})

ament_package()
