cmake_minimum_required(VERSION 3.8)
project(rl_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kvoy_msgs REQUIRED)
find_package(robot_kinematics REQUIRED)

find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  PATHS /usr/include /usr/local/include /usr/include/aarch64-linux-gnu /usr/include/x86_64-linux-gnu
)
find_library(TENSORRT_LIB nvinfer
  PATHS /usr/lib /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu /usr/local/lib
)
find_path(CUDART_INCLUDE_DIR cuda_runtime_api.h
  PATHS
    /usr/local/cuda/include
    /usr/local/cuda/targets/aarch64-linux/include
    /usr/local/cuda/targets/x86_64-linux/include
    /usr/local/cuda-12.1/targets/x86_64-linux/include
    /usr/local/cuda-12.4/targets/x86_64-linux/include
    /usr/local/cuda-12.9/targets/x86_64-linux/include
)
find_library(CUDART_LIB cudart
  PATHS
    /usr/lib
    /usr/lib/aarch64-linux-gnu
    /usr/lib/x86_64-linux-gnu
    /usr/local/cuda/lib64
    /usr/local/cuda/targets/aarch64-linux/lib
    /usr/local/cuda/targets/x86_64-linux/lib
    /usr/local/cuda-12.1/targets/x86_64-linux/lib
    /usr/local/cuda-12.4/targets/x86_64-linux/lib
    /usr/local/cuda-12.9/targets/x86_64-linux/lib
)

if(NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_LIB OR NOT CUDART_INCLUDE_DIR OR NOT CUDART_LIB)
  message(FATAL_ERROR "TensorRT/CUDA runtime not found. Install TensorRT dev packages and CUDA runtime.")
endif()

add_library(rl_controller_node SHARED src/rl_controller_node.cpp)

target_include_directories(rl_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${TENSORRT_INCLUDE_DIR}
  ${CUDART_INCLUDE_DIR}
)

target_link_libraries(rl_controller_node ${TENSORRT_LIB} ${CUDART_LIB})
ament_target_dependencies(rl_controller_node rclcpp kvoy_msgs robot_kinematics)

set_property(TARGET rl_controller_node PROPERTY CXX_STANDARD 17)

install(TARGETS rl_controller_node
  EXPORT rl_controller_node_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(rl_controller_node_export HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp kvoy_msgs robot_kinematics)
ament_package()
