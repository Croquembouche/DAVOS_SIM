# Compiler and flags
CXX := g++
CXXFLAGS := -std=c++11 -I/usr/local/include/opencv4 -I/home/orin/Github/DAVOS/library/unitree_lidar_sdk/include

# OpenCV libraries
LIBS := -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_videoio \
        -lopencv_cudaimgproc -lopencv_imgproc -lopencv_cudawarping -pthread -L/home/orin/Github/DAVOS/library/unitree_lidar_sdk/lib/aarch64

# Additional libraries
LIBS += -ldavos_shm -lunitree_lidar_sdk

# Source files and targets
SRCS := example_lidar_writer.cpp 
TARGETS := example_lidar_writer

# Build all targets
all: $(TARGETS)

# Build each target
example_lidar_writer: example_lidar_writer.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(LIBS)
	
# example_lidar_reader: example_lidar_reader.cpp
# 	$(CXX) $(CXXFLAGS) -o $@ $< $(LIBS)

# Clean target
clean:
	rm -f $(TARGETS)
