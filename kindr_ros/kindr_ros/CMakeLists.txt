cmake_minimum_required(VERSION 3.12)
project(kindr_ros)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(kindr QUIET)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif()

find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${kindr_INCLUDE_DIRS}
)

ament_export_include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

ament_export_dependencies(
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

ament_export_libraries(
  kindr
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()