cmake_minimum_required(VERSION 3.8)
project(point_cloud_subscriber_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(point_cloud_subscriber_node src/point_cloud_subscriber_node.cpp)

ament_target_dependencies(
  point_cloud_subscriber_node
  rclcpp
  sensor_msgs
)

install(TARGETS
  point_cloud_subscriber_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
