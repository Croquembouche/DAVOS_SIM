#include "SharedImage.h"
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <errno.h>
#include <sys/stat.h>

SharedImageReader::SharedImageReader(const std::string &name, int height, int width, int channels, const std::string &data_type)
    : shm_name_(name), shm_fd_(-1), shm_ptr_(nullptr), channels_(channels) {
    if (data_type == "CV_8U") data_size_ = 1;
    else if (data_type == "CV_16U") data_size_ = 2;
    else if (data_type == "CV_32F") data_size_ = 4;
    else if (data_type == "CV_64F") data_size_ = 8;
    else data_size_ = 1;

    size_t img_bytes = height * width * channels * data_size_;
    size_t shm_size = sizeof(SharedImage) - sizeof(unsigned char) + 2 * img_bytes;

    shm_fd_ = shm_open(shm_name_.c_str(), O_RDWR, 0666);
    if (shm_fd_ == -1) {
        std::cerr << "Failed to open shared memory for reading. Error: " << strerror(errno) << std::endl;
        return;
    }
    shm_ptr_ = static_cast<SharedImage*>(mmap(0, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0));
    if (shm_ptr_ == MAP_FAILED) {
        std::cerr << "Failed to map shared memory for reading. Error: " << strerror(errno) << std::endl;
        close(shm_fd_);
        shm_fd_ = -1;
        return;
    }
}

SharedImageReader::~SharedImageReader() {
    if (shm_ptr_ && shm_ptr_ != MAP_FAILED) {
        size_t shm_size = sizeof(SharedImage) - sizeof(unsigned char) + 2 * shm_ptr_->size;
        munmap(shm_ptr_, shm_size);
    }
    if (shm_fd_ != -1) {
        close(shm_fd_);
    }
}

bool SharedImageReader::isInitialized() const {
    return shm_ptr_ != nullptr && shm_ptr_ != MAP_FAILED;
}

bool SharedImageReader::readImage(cv::Mat &image, int &last_frame) {
    if (!isInitialized()) return false;

    int idx = shm_ptr_->front_idx.load(std::memory_order_acquire);
    int frame = shm_ptr_->frame[idx];

    if (frame == last_frame) return false; // No new frame

    // Read the image
    image = cv::Mat(shm_ptr_->height, shm_ptr_->width, CV_8UC3, shm_ptr_->data[idx]).clone();
    last_frame = frame;
    return true;
}

long SharedImageReader::getTimeStamp() {
    int idx = shm_ptr_->front_idx.load(std::memory_order_acquire);
    return shm_ptr_->timeStamp[idx];
}

int SharedImageReader::getFrameNumber() {
    int idx = shm_ptr_->front_idx.load(std::memory_order_acquire);
    return shm_ptr_->frame[idx];
}
