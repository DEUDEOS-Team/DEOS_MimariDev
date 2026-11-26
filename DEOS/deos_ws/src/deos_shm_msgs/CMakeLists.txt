cmake_minimum_required(VERSION 3.16)
project(deos_shm_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FixedFrame640x480.msg"
  "msg/FixedPointCloud64k.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
