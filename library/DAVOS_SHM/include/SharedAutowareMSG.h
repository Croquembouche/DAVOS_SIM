#ifndef SHARED_AUTOWARE_MSG_H
#define SHARED_AUTOWARE_MSG_H

#include <atomic>
#include <string>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>

// Structure for VelocityReport shared memory (double buffer)
struct SharedVelocityReport {
    std::atomic<int> front_idx; // 0 or 1: which buffer is current
    int frame[2];
    long timeStamp[2]; // Timestamp in nanoseconds
    
    // Raw data for VelocityReport
    struct VelocityReportData {
        // Header data
        uint32_t frame_id_length;
        char frame_id[256]; // Fixed buffer for frame_id string
        
        // VelocityReport data
        float longitudinal_velocity;
        float lateral_velocity;
        float heading_rate;
    } velocity_data[2];
};

// Structure for SteeringReport shared memory (double buffer)
struct SharedSteeringReport {
    std::atomic<int> front_idx; // 0 or 1: which buffer is current
    int frame[2];
    long timeStamp[2]; // Timestamp in nanoseconds
    
    // Raw data for SteeringReport
    struct SteeringReportData {
        // Timestamp data (instead of header)
        int32_t sec;
        uint32_t nanosec;
        
        // SteeringReport data
        float steering_tire_angle;
    } steering_data[2];
};

// VelocityReport Writer Class
class SharedVelocityReportWriter {
public:
    SharedVelocityReportWriter(const std::string &name);
    ~SharedVelocityReportWriter();
    bool writeVelocityReport(const autoware_auto_vehicle_msgs::msg::VelocityReport &report);

private:
    std::string shm_name_;
    int shm_fd_;
    SharedVelocityReport *shm_ptr_;
};

// VelocityReport Reader Class
class SharedVelocityReportReader {
public:
    SharedVelocityReportReader(const std::string &name);
    ~SharedVelocityReportReader();
    bool readVelocityReport(autoware_auto_vehicle_msgs::msg::VelocityReport &report, int &last_frame);
    bool isInitialized() const;
    long getTimeStamp();
    int getFrameNumber();

private:
    std::string shm_name_;
    int shm_fd_;
    SharedVelocityReport *shm_ptr_;
};

// SteeringReport Writer Class
class SharedSteeringReportWriter {
public:
    SharedSteeringReportWriter(const std::string &name);
    ~SharedSteeringReportWriter();
    bool writeSteeringReport(const autoware_auto_vehicle_msgs::msg::SteeringReport &report);

private:
    std::string shm_name_;
    int shm_fd_;
    SharedSteeringReport *shm_ptr_;
};

// SteeringReport Reader Class
class SharedSteeringReportReader {
public:
    SharedSteeringReportReader(const std::string &name);
    ~SharedSteeringReportReader();
    bool readSteeringReport(autoware_auto_vehicle_msgs::msg::SteeringReport &report, int &last_frame);
    bool isInitialized() const;
    long getTimeStamp();
    int getFrameNumber();

private:
    std::string shm_name_;
    int shm_fd_;
    SharedSteeringReport *shm_ptr_;
};

#endif // SHARED_AUTOWARE_MSG_H
