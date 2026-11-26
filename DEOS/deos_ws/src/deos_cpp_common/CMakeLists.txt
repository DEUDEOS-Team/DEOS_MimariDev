cmake_minimum_required(VERSION 3.16)
project(deos_cpp_common)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(deos_shm_msgs REQUIRED)

# --- Header-only ortak altyapı kütüphanesi ---
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} INTERFACE
  rclcpp::rclcpp
)

# --- Gecikme & Zero-Copy doğrulama / öğrenme aracı ---
add_executable(shm_zero_copy_bench src/shm_zero_copy_bench.cpp)
target_link_libraries(shm_zero_copy_bench
  ${PROJECT_NAME}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${deos_shm_msgs_TARGETS}
)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(shm_zero_copy_bench PRIVATE
    -Wall -Wextra -Wpedantic -Wconversion -Wshadow
  )
endif()

install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
)

install(TARGETS
  shm_zero_copy_bench
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_deos_cpp_common test/test_deos_cpp_common.cpp)
  target_link_libraries(test_deos_cpp_common
    ${PROJECT_NAME}
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
    ${deos_shm_msgs_TARGETS}
  )
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(test_deos_cpp_common PRIVATE
      -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror
    )
  endif()
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  deos_shm_msgs
)

ament_package()
