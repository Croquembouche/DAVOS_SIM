cmake_minimum_required(VERSION 3.8)
project(image_shm)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add publisher executable
add_executable(image_publisher src/image_pub.cpp)
ament_target_dependencies(image_publisher rclcpp sensor_msgs)

# Add subscriber executable
add_executable(image_subscriber src/image_sub.cpp)
ament_target_dependencies(image_subscriber rclcpp sensor_msgs)

# Install executables
install(TARGETS
  image_publisher
  image_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# Install other files
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

# ament package macros
ament_package()
