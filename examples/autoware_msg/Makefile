# Makefile for Autoware shared memory examples

# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -g
LDFLAGS = -L$(DAVOS_SHM_LIB_DIR) -ldavos_shm -lrt -pthread

# Autoware and ROS2 dependencies
ROS2_INCLUDE = $(shell ros2 pkg-config --cflags rclcpp)
ROS2_LIBS = $(shell ros2 pkg-config --libs rclcpp)

# Include directories
DAVOS_SHM_INCLUDE_DIR = ../../library/DAVOS_SHM/include
DAVOS_SHM_LIB_DIR = ../../library/DAVOS_SHM

# Source files
WRITER_SRC = example_autowaremsg_writer.cpp
READER_SRC = example_autowaremsg_reader.cpp

# Output executables
WRITER_EXE = example_autowaremsg_writer
READER_EXE = example_autowaremsg_reader

# Build targets
.PHONY: all clean

all: $(WRITER_EXE) $(READER_EXE)

$(WRITER_EXE): $(WRITER_SRC)
	$(CXX) $(CXXFLAGS) -I$(DAVOS_SHM_INCLUDE_DIR) $(ROS2_INCLUDE) -o $@ $< $(LDFLAGS) $(ROS2_LIBS)

$(READER_EXE): $(READER_SRC)
	$(CXX) $(CXXFLAGS) -I$(DAVOS_SHM_INCLUDE_DIR) $(ROS2_INCLUDE) -o $@ $< $(LDFLAGS) $(ROS2_LIBS)

clean:
	rm -f $(WRITER_EXE) $(READER_EXE)
