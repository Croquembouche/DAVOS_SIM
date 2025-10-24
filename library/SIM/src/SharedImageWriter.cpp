#include "SharedImage.h"
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <errno.h>
#include <sys/stat.h>
#include <time.h>

SharedImageWriter::SharedImageWriter(const std::string &name, int height, int width, int channels, const std::string &data_type)
    : shm_name_(name), shm_fd_(-1), shm_ptr_(nullptr), channels_(channels) {
    if (data_type == "CV_8U") data_size_ = 1;
    else if (data_type == "CV_16U") data_size_ = 2;
    else if (data_type == "CV_32F") data_size_ = 4;
    else if (data_type == "CV_64F") data_size_ = 8;
    else data_size_ = 1;

    size_t img_bytes = height * width * channels * data_size_;
    size_t shm_size = sizeof(SharedImage) - sizeof(unsigned char) + 2 * img_bytes;

    shm_fd_ = shm_open(shm_name_.c_str(), O_CREAT | O_RDWR, 0666);
    if (shm_fd_ == -1) {
        std::cerr << "Failed to create shared memory. Error: " << strerror(errno) << std::endl;
        return;
    }
    if (ftruncate(shm_fd_, shm_size) == -1) {
        std::cerr << "Failed to set size of shared memory. Error: " << strerror(errno) << std::endl;
        close(shm_fd_);
        shm_fd_ = -1;
        return;
    }
    shm_ptr_ = static_cast<SharedImage*>(mmap(0, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0));
    if (shm_ptr_ == MAP_FAILED) {
        std::cerr << "Failed to map shared memory. Error: " << strerror(errno) << std::endl;
        close(shm_fd_);
        shm_fd_ = -1;
        return;
    }

    shm_ptr_->front_idx.store(0, std::memory_order_relaxed);
    shm_ptr_->frame[0] = shm_ptr_->frame[1] = 0;
    shm_ptr_->timeStamp[0] = shm_ptr_->timeStamp[1] = 0;
    shm_ptr_->size = img_bytes;
    shm_ptr_->height = height;
    shm_ptr_->width = width;
}

SharedImageWriter::~SharedImageWriter() {
    if (shm_ptr_ && shm_ptr_ != MAP_FAILED) {
        size_t shm_size = sizeof(SharedImage) - sizeof(unsigned char) + 2 * shm_ptr_->size;
        munmap(shm_ptr_, shm_size);
    }
    if (shm_fd_ != -1) {
        close(shm_fd_);
        shm_unlink(shm_name_.c_str());
    }
}

bool SharedImageWriter::writeImage(const cv::Mat &image) {
    if (!shm_ptr_) return false;
    if (image.empty() || image.total() * image.elemSize() != shm_ptr_->size) return false;

    int next_idx = 1 - shm_ptr_->front_idx.load(std::memory_order_acquire);

    // Write to the non-front buffer
    memcpy(shm_ptr_->data[next_idx], image.data, shm_ptr_->size);

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    shm_ptr_->timeStamp[next_idx] = ts.tv_sec * 1000000000L + ts.tv_nsec;
    shm_ptr_->frame[next_idx] = shm_ptr_->frame[1 - next_idx] + 1;

    // Atomically publish the new buffer
    shm_ptr_->front_idx.store(next_idx, std::memory_order_release);

    return true;
}
