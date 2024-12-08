cmake_minimum_required(VERSION 3.5)

project(rmw_davos_dds)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Set shared library build globally
set(BUILD_SHARED_LIBS ON)

find_package(ament_cmake REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_c REQUIRED)
find_package(rosidl_typesupport_interface REQUIRED)


# Specify OpenCV manually since it is built from source
set(OpenCV_DIR "/usr/local/lib/cmake/opencv4")  # Path to OpenCV's CMake configuration files
find_package(OpenCV REQUIRED)

# Add the shared library and source files
add_library(${PROJECT_NAME} SHARED
  src/davos_dds_init.cpp
  src/davos_dds_create_publisher.cpp
  src/davos_dds_publish.cpp
  src/davos_dds_create_subscription.cpp
  src/davos_dds_take.cpp
  src/rmw_identifier.cpp
)

# Specify include directories for this target
target_include_directories(${PROJECT_NAME}
  PUBLIC
  include                          # Package-specific includes
  ${rmw_INCLUDE_DIRS}              # RMW headers
  ${rcutils_INCLUDE_DIRS}          # RCUtils headers
  ${rosidl_typesupport_interface_INCLUDE_DIRS}  # rosidl_typesupport_interface headers
  /usr/local/include/opencv4       # OpenCV headers
  /usr/local/include               # SharedImage.h include
)

# Link the shared memory library and other dependencies
target_link_libraries(${PROJECT_NAME}
  shared_memory_image
  ${OpenCV_LIBS}                   # OpenCV libraries
)

# Install the shared library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

# Export dependencies for downstream packages
ament_export_dependencies(rmw rcutils)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# Package setup for ROS 2
ament_package()
