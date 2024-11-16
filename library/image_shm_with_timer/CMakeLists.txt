# Minimum version of CMake required
cmake_minimum_required(VERSION 3.10)

# Project name and version
project(shared_memory_image VERSION 1.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include the directory with header files
include_directories(include)

# Add the source files to the library
add_library(shared_memory_image SHARED src/SharedImageWriter.cpp src/SharedImageReader.cpp)

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Link the OpenCV library
target_link_libraries(shared_memory_image PUBLIC ${OpenCV_LIBS})

# Install the library and headers
install(TARGETS shared_memory_image DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)


