# Compiler and flags
CXX := g++
NVCC := nvcc  # CUDA Compiler

# Compiler flags for includes
CXXFLAGS := -std=c++14 \
            -I/usr/local/include/opencv4 \
            -I/media/nvidia/AGXDavos/Github/DAVOS_SIM/library/unitree_lidar_sdk/include \
            -I/media/nvidia/AGXDavos/Github/DAVOS_SIM/library/DAVOS_SHM/include \
            -I/usr/include/pcl-1.10/ \
            -I/usr/include/eigen3 \
            -fopenmp  # Enable OpenMP

# CUDA flags for NVCC (to link with the CUDA toolkit and set architecture)
NVCCFLAGS := -O3 -arch=sm_72 -std=c++14 -Xcompiler "-fopenmp -pthread" \
             -I/usr/local/include/opencv4 \
             -I/media/nvidia/AGXDavos/Github/DAVOS_SIM/library/unitree_lidar_sdk/include \
             -I/media/nvidia/AGXDavos/Github/DAVOS_SIM/library/DAVOS_SHM/include \
             -I/usr/include/pcl-1.10/ \
             -I/usr/include/eigen3 
# OpenCV and PCL libraries (libraries linked at the end)
LIBS := -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_videoio \
        -lopencv_cudaimgproc -lopencv_imgproc -lopencv_cudawarping \
        -L/media/nvidia/AGXDavos/Github/DAVOS_SIM/library/unitree_lidar_sdk/lib/aarch64 

# Add the CUDA library path and link with libcudart
LIBS += -L/usr/local/cuda-11.4/targets/aarch64-linux/lib -lcudart

# Additional libraries (unitree, davos, pcl)
LIBS += -ldavos_shm -lunitree_lidar_sdk \
        -lpcl_common -lpcl_io -lpcl_filters -lpcl_kdtree

# Source files and targets
SRCS := example_lidar_writer.cpp example_lidar_writer test_lidar_reader.cpp
TARGETS := example_lidar_writer example_lidar_reader test_lidar_reader

# Build all targets
all: $(TARGETS)

# Build each target
example_lidar_writer: example_lidar_writer.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(LIBS)

example_lidar_reader: example_lidar_reader.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(LIBS)

# Build the new CUDA-OpenMP enabled test_lidar_reader target
# test_lidar_reader: test_lidar_reader.cu
# 	$(NVCC) $(NVCCFLAGS) -o $@ $< $(LIBS)

# Clean target
clean:
	rm -f $(TARGETS)
