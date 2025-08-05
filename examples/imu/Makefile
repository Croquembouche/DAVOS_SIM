# Compiler and flags
CXX=g++
CXXFLAGS=-std=c++14 -I/opt/ros/humble/include -I/opt/ros/humble/include/geometry_msgs -I/opt/ros/humble/include/rosidl_runtime_cpp -I/opt/ros/humble/include/rosidl_runtime_c -I/opt/ros/humble/include/std_msgs -I/opt/ros/humble/include/builtin_interfaces -I/opt/ros/humble/include/rosidl_typesupport_interface -I/usr/local/include
LDFLAGS=-L/usr/local/lib -ldavos_shm

# Build all targets
all: example_imu_reader

# Build each target
example_imu_reader: example_imu_reader.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS)

# Clean target
clean:
	rm -f example_imu_reader

.PHONY: all clean
