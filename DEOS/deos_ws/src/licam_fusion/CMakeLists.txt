cmake_minimum_required(VERSION 3.16)
project(licam_fusion)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(deos_cpp_common REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

# --- Shared library for composable node ---
add_library(fusion_component SHARED src/fusion_node.cpp)

target_include_directories(fusion_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(fusion_component PRIVATE
    -Wall -Wextra -Wpedantic -Wconversion -Wshadow
  )
endif()

target_link_libraries(fusion_component PUBLIC
  Eigen3::Eigen
  deos_cpp_common::deos_cpp_common
  rclcpp::rclcpp
  rclcpp_components::component
  diagnostic_updater::diagnostic_updater
  message_filters::message_filters
  tf2_ros::tf2_ros
  tf2_eigen::tf2_eigen
  ${diagnostic_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
  ${vision_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
)

# Register the node as a composable component for zero-copy intra-process communication
rclcpp_components_register_node(fusion_component
  PLUGIN "licam_fusion::FusionNode"
  EXECUTABLE fusion_node
)

install(TARGETS
  fusion_component
  EXPORT export_fusion_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_fusion_core test/test_fusion_core.cpp)
  target_include_directories(test_fusion_core PRIVATE include)
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(test_fusion_core PRIVATE
      -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror
    )
  endif()

  ament_add_gtest(test_fusion_node test/test_fusion_node.cpp)
  target_link_libraries(test_fusion_node
    fusion_component
    tf2_ros::static_transform_broadcaster_node
  )
endif()

ament_export_include_directories(include)
ament_export_targets(export_fusion_component HAS_LIBRARY_TARGET)
ament_export_dependencies(
  deos_cpp_common
  rclcpp
  rclcpp_components
  diagnostic_updater
  diagnostic_msgs
  message_filters
  vision_msgs
  visualization_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  Eigen3
)

ament_package()
