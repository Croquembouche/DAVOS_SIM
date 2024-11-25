# Compiler and flags
CXX := g++
CXXFLAGS := -std=c++11 -I /usr/local/include/opencv4

# OpenCV libraries
LIBS := -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_videoio \
        -lopencv_cudaimgproc -lopencv_imgproc -lopencv_cudawarping -pthread

# Additional libraries
LIBS += -lshared_memory_image

# Source files and targets
SRCS := example_camera_writer.cpp
TARGETS := example_camera_writer

# Build all targets
all: $(TARGETS)

# Build each target
example_camera_writer: example_camera_writer.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(LIBS)

# Clean target
clean:
	rm -f $(TARGETS)
