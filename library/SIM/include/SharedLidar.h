#ifndef SHARED_LIDAR_H
#define SHARED_LIDAR_H

#include <atomic>
#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// Structure defining the layout of shared memory (double buffer)
struct SharedLidar {
    std::atomic<int> front_idx; // 0 or 1: which buffer is current
    int frame[2];
    long timeStamp[2];
    unsigned long num_points;
    unsigned long max_points;
    pcl::PointXYZ data[2][1]; // Flexible array member, real size set at allocation
};

class SharedLidarWriter {
public:
    SharedLidarWriter(const std::string &name, size_t max_points);
    ~SharedLidarWriter();
    bool writePointCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud);

private:
    std::string shm_name_;
    int shm_fd_;
    SharedLidar *shm_ptr_;
    size_t max_points_;
};

class SharedLidarReader {
public:
    SharedLidarReader(const std::string &name, size_t max_points);
    ~SharedLidarReader();
    bool readPointCloud(pcl::PointCloud<pcl::PointXYZ> &cloud, int &last_frame);
    bool isInitialized() const;
    long getTimeStamp();
    int getFrameNumber();

private:
    std::string shm_name_;
    int shm_fd_;
    SharedLidar *shm_ptr_;
    size_t max_points_;
};

#endif // SHARED_LIDAR_H
