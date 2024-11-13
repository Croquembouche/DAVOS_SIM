#ifndef SHARED_IMAGE_H
#define SHARED_IMAGE_H

#include <string>
#include <opencv2/opencv.hpp>
#include <condition_variable>
#include <atomic>
#include <mutex>

struct SharedImage {
    int version;                   // Version to indicate when new data is available
    unsigned char data[921600];    // Image data buffer (640x480 RGB)
    std::atomic<int> reader_count; // Atomic counter to track active readers
    std::condition_variable_any cv; // Condition variable to notify when it's safe to write
    std::mutex cv_mutex;           // Mutex for use with the condition variable
};

// Writer class for a single producer
class SharedImageWriter {
public:
    SharedImageWriter(const std::string &name); // Constructor to initialize shared memory
    ~SharedImageWriter();                       // Destructor to clean up shared memory

    bool writeImage(const cv::Mat &image);      // Function to write an image to shared memory

private:
    std::string shm_name_;                      // Shared memory segment name
    int shm_fd_;                                // Shared memory file descriptor
    SharedImage *shm_ptr_;                      // Pointer to the mapped shared memory
};

// Reader class for multiple consumers
class SharedImageReader {
public:
    SharedImageReader(const std::string &name); // Constructor to initialize shared memory
    ~SharedImageReader();                       // Destructor to clean up shared memory

    bool readImage(cv::Mat &image, int &last_version); // Function to read image from shared memory

private:
    std::string shm_name_;                      // Shared memory segment name
    int shm_fd_;                                // Shared memory file descriptor
    SharedImage *shm_ptr_;                      // Pointer to the mapped shared memory
};

#endif // SHARED_IMAGE_H



