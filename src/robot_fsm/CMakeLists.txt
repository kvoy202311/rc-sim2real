cmake_minimum_required(VERSION 3.8)
project(robot_fsm)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kvoy_msgs REQUIRED)
find_package(robot_kinematics REQUIRED)
find_package(rl_controller REQUIRED)

add_executable(robot_fsm_node src/robot_fsm_node.cpp)

target_include_directories(robot_fsm_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(robot_fsm_node
  robot_kinematics::kinematics_node
  rl_controller::rl_controller_node
)

ament_target_dependencies(robot_fsm_node rclcpp kvoy_msgs robot_kinematics rl_controller)

set_property(TARGET robot_fsm_node PROPERTY CXX_STANDARD 17)

install(TARGETS robot_fsm_node DESTINATION lib/${PROJECT_NAME})

ament_package()
