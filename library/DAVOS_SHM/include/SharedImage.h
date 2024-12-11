#ifndef SHARED_IMAGE_H
#define SHARED_IMAGE_H

#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <atomic>
#include <pthread.h>

// Structure defining the layout of shared memory
struct SharedImage{
    int frame;                       // Frame version
    long timeStamp;                  // TimeStamp
    unsigned long size;
    std::atomic<bool> ready_to_read; // Flag to indicate if readers can proceed
    std::atomic<int> waiting_readers; // Number of readers waiting for the writer
    pthread_mutex_t mutex;           // Mutex for synchronization
    pthread_cond_t read_cv;          // Condition variable to signal readers
    unsigned char data[];      // Single buffer for image data
};

// Writer class for a single producer
class SharedImageWriter{
public:
    SharedImageWriter(const std::string &name, const int height, const int width, const int channels, const std::string &data_type); // Constructor to initialize shared memory
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
    SharedImageReader(const std::string &name, const int height, const int width, const int channels, const std::string &data_type); // Constructor to initialize shared memory
    ~SharedImageReader();                       // Destructor to clean up shared memory
    bool readImage(cv::Mat &image, int &last_frame); // Function to read image from shared memory
    bool isInitialized() const;                 // Check if initialization was successful
    long getTimeStamp();                        // get Image timeStamp
    int getFrameNumber();                       // get Frame Number
private:
    std::string shm_name_;                      // Shared memory segment name
    int shm_fd_;                                // Shared memory file descriptor
    SharedImage *shm_ptr_;                      // Pointer to the mapped shared memory
};

#endif // SHARED_IMAGE_H
