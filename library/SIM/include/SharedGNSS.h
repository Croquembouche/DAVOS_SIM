#ifndef SHARED_GNSS_H
#define SHARED_GNSS_H

#include <atomic>
#include <string>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

// Structure defining the layout of shared memory (double buffer)
struct SharedGNSS {
    std::atomic<int> front_idx; // 0 or 1: which buffer is current
    int frame[2];
    long timeStamp[2]; // Timestamp in nanoseconds
    
    // Raw data for GNSS rather than ROS2 message objects
    struct GNSSData {
        // Header data
        uint32_t frame_id_length;
        char frame_id[256]; // Fixed buffer for frame_id string
        
        // Pose data
        double position_x, position_y, position_z;
        double orientation_x, orientation_y, orientation_z, orientation_w;
        
        // Covariance (6x6 matrix = 36 elements)
        double covariance[36];
    } gnss_data[2];
};

class SharedGNSSWriter {
public:
    SharedGNSSWriter(const std::string &name);
    ~SharedGNSSWriter();
    bool writeGNSS(const geometry_msgs::msg::PoseWithCovarianceStamped &gnssMsg);

private:
    std::string shm_name_;
    int shm_fd_;
    SharedGNSS *shm_ptr_;
};

class SharedGNSSReader {
public:
    SharedGNSSReader(const std::string &name);
    ~SharedGNSSReader();
    bool readGNSS(geometry_msgs::msg::PoseWithCovarianceStamped &gnssMsg, int &last_frame);
    bool isInitialized() const;
    long getTimeStamp();
    int getFrameNumber();

private:
    std::string shm_name_;
    int shm_fd_;
    SharedGNSS *shm_ptr_;
};
#endif
