cmake_minimum_required(VERSION 3.5)
project(lidar_merge)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

# Executable 1: lidar_merge
add_executable(lidar_merge src/lidar_merge.cpp)
ament_target_dependencies(lidar_merge
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  pcl_conversions
  pcl_ros
  geometry_msgs
  PCL
  Eigen3
)

# Executable 2: lidar_merge_filter
add_executable(lidar_merge_filter src/lidar_merge_filter.cpp)
ament_target_dependencies(lidar_merge_filter
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  pcl_conversions
  pcl_ros
  geometry_msgs
  PCL
  Eigen3
)

# Install executables
install(TARGETS
  lidar_merge
  lidar_merge_filter
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (if any)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
