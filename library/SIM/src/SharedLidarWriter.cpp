#include "SharedLidar.h"
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <ctime>

SharedLidarWriter::SharedLidarWriter(const std::string &name, size_t max_points)
    : shm_name_(name), shm_fd_(-1), shm_ptr_(nullptr), max_points_(max_points) {
    size_t shm_size = sizeof(SharedLidar) - sizeof(pcl::PointXYZ) + 2 * max_points_ * sizeof(pcl::PointXYZ);
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
    shm_ptr_ = static_cast<SharedLidar*>(mmap(0, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0));
    if (shm_ptr_ == MAP_FAILED) {
        std::cerr << "Failed to map shared memory. Error: " << strerror(errno) << std::endl;
        close(shm_fd_);
        shm_fd_ = -1;
        return;
    }
    shm_ptr_->front_idx.store(0, std::memory_order_relaxed);
    shm_ptr_->frame[0] = shm_ptr_->frame[1] = 0;
    shm_ptr_->timeStamp[0] = shm_ptr_->timeStamp[1] = 0;
    shm_ptr_->num_points = 0;
    shm_ptr_->max_points = max_points_;
}

SharedLidarWriter::~SharedLidarWriter() {
    if (shm_ptr_ && shm_ptr_ != MAP_FAILED) {
        size_t shm_size = sizeof(SharedLidar) - sizeof(pcl::PointXYZ) + 2 * max_points_ * sizeof(pcl::PointXYZ);
        munmap(shm_ptr_, shm_size);
    }
    if (shm_fd_ != -1) {
        close(shm_fd_);
        shm_unlink(shm_name_.c_str());
    }
}

bool SharedLidarWriter::writePointCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud) {
    if (!shm_ptr_) return false;
    if (cloud.points.size() > max_points_) return false;
    int next_idx = 1 - shm_ptr_->front_idx.load(std::memory_order_acquire);
    // Copy point cloud data directly (no serialization overhead)
    memcpy(shm_ptr_->data[next_idx], cloud.points.data(), cloud.points.size() * sizeof(pcl::PointXYZ));
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    shm_ptr_->timeStamp[next_idx] = ts.tv_sec * 1000000000L + ts.tv_nsec;
    shm_ptr_->frame[next_idx] = shm_ptr_->frame[1 - next_idx] + 1;
    shm_ptr_->num_points = cloud.points.size();
    // Atomically publish the new buffer
    shm_ptr_->front_idx.store(next_idx, std::memory_order_release);
    return true;
}
