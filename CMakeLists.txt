cmake_minimum_required(VERSION 3.14)
project(pointcloud_publisher)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# Create the executable
add_executable(pointcloud_publisher_node src/pointcloud_publisher_node.cpp)

# Link against the necessary libraries
ament_target_dependencies(pointcloud_publisher_node rclcpp sensor_msgs pcl_conversions pcl_ros)


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


# Install the executable
install(TARGETS
  pointcloud_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
