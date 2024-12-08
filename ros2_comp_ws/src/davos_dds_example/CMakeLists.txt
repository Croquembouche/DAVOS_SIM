cmake_minimum_required(VERSION 3.10)
project(davos_dds_example)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Define the RMW implementation
if(NOT DEFINED RMW_IMPLEMENTATION)
  set(RMW_IMPLEMENTATION "rmw_davos_dds")
endif()
message(STATUS "Using RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION}")

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_davos_dds REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)


# Include directories
include_directories(
  include
  ${rosidl_runtime_cpp_INCLUDE_DIRS}
)

# Define the executable
add_executable(davos_dds_publisher src/publisher.cpp)

# Link libraries
target_link_libraries(davos_dds_publisher ${OpenCV_LIBS})
ament_target_dependencies(davos_dds_publisher rclcpp rmw_davos_dds OpenCV)

# Install the executable
install(TARGETS davos_dds_publisher
  DESTINATION lib/${PROJECT_NAME})

# Install other files (if any)
install(DIRECTORY include/
  DESTINATION include)

ament_package()
