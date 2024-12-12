# Compiler and flags
CXX := g++

# Compiler flags for includes
CXXFLAGS := -std=c++14 \
            -I/usr/local/include/opencv4 \
            -I/home/orin/Github/DAVOS/library/unitree_lidar_sdk/include \
            -I/usr/include/pcl-1.12/ \
            -I/usr/include/eigen3 
			
# OpenCV and PCL libraries (libraries linked at the end)
LIBS := -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_videoio \
        -lopencv_cudaimgproc -lopencv_imgproc -lopencv_cudawarping -pthread \
        -L/home/orin/Github/DAVOS/library/unitree_lidar_sdk/lib/aarch64 

# Additional libraries (unitree, davos, pcl)
LIBS += -ldavos_shm -lunitree_lidar_sdk \
        -lpcl_common -lpcl_io -lpcl_filters -lpcl_kdtree

# Source files and targets
SRCS := example_lidar_writer.cpp example_lidar_writer
TARGETS := example_lidar_writer example_lidar_reader

# Build all targets
all: $(TARGETS)

# Build each target
example_lidar_writer: example_lidar_writer.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(LIBS)
	
example_lidar_reader: example_lidar_reader.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(LIBS)

# Clean target
clean:
	rm -f $(TARGETS)
