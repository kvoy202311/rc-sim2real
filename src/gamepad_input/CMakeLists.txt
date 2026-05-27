cmake_minimum_required(VERSION 3.8)
project(gamepad_input)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kvoy_msgs REQUIRED)
find_package(SDL2 REQUIRED)

add_executable(gamepad_input_node src/gamepad_input_node.cpp)

target_include_directories(gamepad_input_node PUBLIC ${SDL2_INCLUDE_DIRS})
target_link_libraries(gamepad_input_node ${SDL2_LIBRARIES})
ament_target_dependencies(gamepad_input_node rclcpp kvoy_msgs)

install(TARGETS gamepad_input_node DESTINATION lib/${PROJECT_NAME})

ament_package()
