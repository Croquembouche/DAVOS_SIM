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
find_package(rmw_implementation_cmake REQUIRED)

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
  src/rmw_init_options.cpp
  src/rmw_context_fini.cpp 
  src/rmw_create_node.cpp 
  src/rmw_destroy_node.cpp 
  src/rmw_shutdown.cpp 
  src/rmw_wait.cpp 
  src/rmw_serialize_deserialize.cpp
  src/davos_dds_services_clients.cpp
)

# Specify include directories for this target
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  # Build-time include path
    $<INSTALL_INTERFACE:include>                           # Install-time include path
    ${rmw_INCLUDE_DIRS}
    ${rcutils_INCLUDE_DIRS}
    ${rosidl_typesupport_interface_INCLUDE_DIRS}
    /usr/local/include/opencv4
    /usr/local/include
)


# Link the shared memory library and other dependencies
target_link_libraries(${PROJECT_NAME}
  shared_memory_image
  ${OpenCV_LIBS}
  ${rmw_LIBRARIES}
  ${rcutils_LIBRARIES}
  ${rosidl_typesupport_introspection_cpp_LIBRARIES}
  ${rosidl_typesupport_introspection_c_LIBRARIES}
)

# Register this RMW implementation with supported typesupports
register_rmw_implementation(
  "c:rosidl_typesupport_c:rosidl_typesupport_introspection_c"
  "cpp:rosidl_typesupport_cpp:rosidl_typesupport_introspection_cpp"
)

# Register resource file
ament_index_register_resource("rmw_implementation")

# Install the shared library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
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
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rmw rcutils rosidl_typesupport_interface)
ament_export_include_directories(
  include
  ${rmw_INCLUDE_DIRS}
  ${rcutils_INCLUDE_DIRS}
  ${rosidl_typesupport_interface_INCLUDE_DIRS}
)
ament_export_libraries(${PROJECT_NAME})

# Package setup for ROS 2
ament_package()
