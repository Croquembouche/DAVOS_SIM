#ifndef SHARED_IMAGE_H
#define SHARED_IMAGE_H

#include <atomic>
#include <string>
#include <opencv2/opencv.hpp>

// Structure defining the layout of shared memory (double buffer)
struct SharedImage {
    std::atomic<int> front_idx; // 0 or 1: which buffer is current
    int frame[2];
    long timeStamp[2];
    unsigned long size;
    int height;
    int width;
    unsigned char data[2][1]; // Flexible array member, real size set at allocation
};

class SharedImageWriter {
public:
    SharedImageWriter(const std::string &name, int height, int width, int channels, const std::string &data_type);
    ~SharedImageWriter();
    bool writeImage(const cv::Mat &image);

private:
    std::string shm_name_;
    int shm_fd_;
    SharedImage *shm_ptr_;
    int channels_;
    int data_size_;
};

class SharedImageReader {
public:
    SharedImageReader(const std::string &name, int height, int width, int channels, const std::string &data_type);
    ~SharedImageReader();
    bool readImage(cv::Mat &image, int &last_frame);
    bool isInitialized() const;
    long getTimeStamp();
    int getFrameNumber();

private:
    std::string shm_name_;
    int shm_fd_;
    SharedImage *shm_ptr_;
    int channels_;
    int data_size_;
};

#endif // SHARED_IMAGE_H
