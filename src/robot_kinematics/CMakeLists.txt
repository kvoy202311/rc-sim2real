cmake_minimum_required(VERSION 3.8)
project(robot_kinematics)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kvoy_msgs REQUIRED)

add_library(kinematics_node SHARED src/kinematics_node.cpp)

target_include_directories(kinematics_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(kinematics_node rclcpp kvoy_msgs)

install(TARGETS kinematics_node
  EXPORT kinematics_node_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(kinematics_node_export HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp kvoy_msgs)
ament_package()
