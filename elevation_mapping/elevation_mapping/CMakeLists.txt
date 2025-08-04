cmake_minimum_required(VERSION 3.12)
project(elevation_mapping)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(kindr REQUIRED)
find_package(kindr_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(message_logger REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

include_directories(
  include  
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

###########
# Library #
###########

# Our custom pcl type with precompiled template instantiations.
add_library(${PROJECT_NAME}_pcl_types
  src/PointXYZRGBConfidenceRatio.cpp
)
ament_target_dependencies(${PROJECT_NAME}_pcl_types
  pcl_ros
  rclcpp
  grid_map_core
)

add_library(${PROJECT_NAME}_library
  src/ElevationMapping.cpp
  src/ElevationMap.cpp
  src/input_sources/Input.cpp
  src/input_sources/InputSourceManager.cpp
  src/postprocessing/PostprocessorPool.cpp
  src/postprocessing/PostprocessingWorker.cpp
  src/postprocessing/PostprocessingPipelineFunctor.cpp
  src/RobotMotionMapUpdater.cpp
  src/sensor_processors/SensorProcessorBase.cpp
  src/sensor_processors/StructuredLightSensorProcessor.cpp
  src/sensor_processors/StereoSensorProcessor.cpp
  src/sensor_processors/LaserSensorProcessor.cpp
  src/sensor_processors/PerfectSensorProcessor.cpp
)
target_link_libraries(${PROJECT_NAME}_library
  ${PROJECT_NAME}_pcl_types
  pthread
)

ament_target_dependencies(${PROJECT_NAME}_library
  grid_map_core
  grid_map_ros
  grid_map_msgs
  kindr
  kindr_ros
  message_filters
  message_logger
  pcl_ros
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  Boost
  Eigen3
)

##############
# Executable #
##############

add_executable(${PROJECT_NAME}
  src/elevation_mapping_node.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_library
)

#############
## Install ##
#############

install(TARGETS 
    ${PROJECT_NAME}
    ${PROJECT_NAME}_pcl_types
    ${PROJECT_NAME}_library
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  grid_map_core
  grid_map_ros
  grid_map_msgs
  kindr
  kindr_ros
  message_filters
  message_logger
  pcl_ros
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  Boost
  Eigen3
)

ament_package()
