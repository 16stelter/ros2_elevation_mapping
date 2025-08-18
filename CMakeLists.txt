cmake_minimum_required(VERSION 3.8)
project(ros2_elevation_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
)

add_executable(ElevationMapping src/elevation_mapping.cpp)

ament_target_dependencies(
  ElevationMapping
  rclcpp
  grid_map_core
  grid_map_ros
  grid_map_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  sensor_msgs
  std_msgs
)

install(TARGETS ElevationMapping
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
