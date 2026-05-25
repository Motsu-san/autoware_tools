cmake_minimum_required(VERSION 3.14)
project(ndt_direct_measure)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Match ndt_omp / convergence_evaluator SIMD flags on x86_64.
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
  add_compile_options(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
elseif(BUILD_WITH_MARCH_NATIVE)
  add_compile_options(-march=native)
endif()

find_package(ament_cmake REQUIRED)
find_package(autoware_ndt_scan_matcher REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(ndt_direct_measure_node
  src/ndt_direct_measure_node.cpp
  src/map_loader.cpp
  src/bag_reader.cpp
  src/pose_yaml.cpp
  src/output_writer.cpp
)

target_include_directories(ndt_direct_measure_node PRIVATE
  include
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(ndt_direct_measure_node
  autoware_ndt_scan_matcher
  geometry_msgs
  pcl_conversions
  rclcpp
  rosbag2_cpp
  sensor_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
)

target_link_libraries(ndt_direct_measure_node
  ${PCL_LIBRARIES}
  yaml-cpp
)

install(TARGETS ndt_direct_measure_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
