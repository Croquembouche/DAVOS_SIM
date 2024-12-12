#ifndef SHARED_LIDAR_H
#define SHARED_LIDAR_H

#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <atomic>
#include <pthread.h>
#include <unitree_lidar_sdk.h>

// Structure defining the layout of shared memory
struct SharedUnitreeLidar{
    int frame;                       // Frame version
    long timeStamp;                  // TimeStamp
    size_t size;
    std::atomic<bool> ready_to_read; // Flag to indicate if readers can proceed
    std::atomic<int> waiting_readers; // Number of readers waiting for the writer
    pthread_mutex_t mutex;           // Mutex for synchronization
    pthread_cond_t read_cv;          // Condition variable to signal readers
    unitree_lidar_sdk::PointCloudUnitree data[];      // Single buffer for Lidar data
};

// Writer class for a single producer
class SharedUnitreeLidarWriter{
public:
    SharedUnitreeLidarWriter(const std::string &name, int cloud_scan_num, int point_byte); // Constructor to initialize shared memory
    ~SharedUnitreeLidarWriter();                       // Destructor to clean up shared memory
    bool writeLidar(unitree_lidar_sdk::PointCloudUnitree &lidar);      // Function to write an Lidar to shared memory

private:
    std::string shm_name_;                      // Shared memory segment name
    int shm_fd_;                                // Shared memory file descriptor
    SharedUnitreeLidar *shm_ptr_;                      // Pointer to the mapped shared memory
};

// Reader class for multiple consumers
class SharedUnitreeLidarReader {
public:
    SharedUnitreeLidarReader(const std::string &name, int cloud_scan_num, int point_byte); // Constructor to initialize shared memory
    ~SharedUnitreeLidarReader();                       // Destructor to clean up shared memory
    bool readLidar(unitree_lidar_sdk::PointCloudUnitree &lidar, int &last_frame); // Function to read Lidar from shared memory
    bool isInitialized() const;                 // Check if initialization was successful
    long getTimeStamp();                        // get Image timeStamp
    int getFrameNumber();                       // get Frame Number
private:
    std::string shm_name_;                      // Shared memory segment name
    int shm_fd_;                                // Shared memory file descriptor
    SharedUnitreeLidar *shm_ptr_;                      // Pointer to the mapped shared memory
};

#endif // SHARED_LIDAR_H
