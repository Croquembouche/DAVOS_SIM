#ifndef SHARED_IMU_H
#define SHARED_IMU_H

#include <atomic>
#include <string>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

// Structure defining the layout of shared memory (double buffer)
struct SharedIMU {
    std::atomic<int> front_idx; // 0 or 1: which buffer is current
    int frame[2];
    long timeStamp[2]; // Timestamp in nanoseconds
    
    // Raw data for IMU rather than ROS2 message objects
    struct IMUData {
        // Header data
        uint32_t frame_id_length;
        char frame_id[256]; // Fixed buffer for frame_id string
        
        // Twist data
        double linear_x, linear_y, linear_z;
        double angular_x, angular_y, angular_z;
        
        // Covariance (6x6 matrix = 36 elements)
        double covariance[36];
    } imu_data[2];
};

class SharedIMUWriter {
public:
    SharedIMUWriter(const std::string &name);
    ~SharedIMUWriter();
    bool writeIMU(const SharedIMU::IMUData &imuData); // Changed to use IMUData struct

private:
    std::string shm_name_;
    int shm_fd_;
    SharedIMU *shm_ptr_;
};

class SharedIMUReader {
public:
    SharedIMUReader(const std::string &name);
    ~SharedIMUReader();
    bool readIMU(SharedIMU::IMUData &imuData, int &last_frame); // Changed to use IMUData struct
    bool isInitialized() const;
    long getTimeStamp();
    int getFrameNumber();

private:
    std::string shm_name_;
    int shm_fd_;
    SharedIMU *shm_ptr_;
};
#endif