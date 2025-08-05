# Minimum version of CMake required
cmake_minimum_required(VERSION 3.10)

# Project name and version
project(davos_shm VERSION 1.0)


# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find PCL (replace 1.10 with your installed version if needed)
find_package(PCL 1.10 REQUIRED)

# Add PCL include directories
include_directories(${PCL_INCLUDE_DIRS})

# Add PCL library directories (optional, usually not needed)
# link_directories(${PCL_LIBRARY_DIRS})

# Add PCL definitions
add_definitions(${PCL_DEFINITIONS})

# Include the directory with header files
include_directories(include)

# Add the source files to the library
add_library(davos_shm SHARED 
    src/SharedImageWriter.cpp 
    src/SharedImageReader.cpp 
    # src/SharedUnitreeLidarWriter.cpp 
    # src/SharedUnitreeLidarReader.cpp
    src/SharedLidarWriter.cpp 
    src/SharedLidarReader.cpp
    )

# Find OpenCV package
find_package(OpenCV REQUIRED)
# include_directories("${CMAKE_SOURCE_DIR}/../unitree_lidar_sdk/include")

# add_library(UnitreeLidarSDK STATIC IMPORTED)
# set_target_properties(UnitreeLidarSDK PROPERTIES
    # IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/../unitree_lidar_sdk/lib/aarch64/libunitree_lidar_sdk.a"
# )

# target_include_directories(UnitreeLidarSDK INTERFACE 
    # "${CMAKE_SOURCE_DIR}/../unitree_lidar_sdk/include"
# )


# Link the libraries
# target_link_libraries(davos_shm PUBLIC ${OpenCV_LIBS} ${PCL_LIBRARIES} UnitreeLidarSDK)
target_link_libraries(davos_shm PUBLIC ${OpenCV_LIBS} ${PCL_LIBRARIES})



# Install the library and headers
install(TARGETS davos_shm DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)


