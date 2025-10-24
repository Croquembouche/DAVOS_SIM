#include "SharedLidar.h"
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

SharedLidarReader::SharedLidarReader(const std::string &name, size_t max_points)
    : shm_name_(name), shm_fd_(-1), shm_ptr_(nullptr), max_points_(max_points) {
    size_t shm_size = sizeof(SharedLidar) - sizeof(pcl::PointXYZ) + 2 * max_points_ * sizeof(pcl::PointXYZ);
    shm_fd_ = shm_open(shm_name_.c_str(), O_RDWR, 0666);
    if (shm_fd_ == -1) {
        std::cerr << "Failed to open shared memory for reading. Error: " << strerror(errno) << std::endl;
        return;
    }
    shm_ptr_ = static_cast<SharedLidar*>(mmap(0, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0));
    if (shm_ptr_ == MAP_FAILED) {
        std::cerr << "Failed to map shared memory for reading. Error: " << strerror(errno) << std::endl;
        close(shm_fd_);
        shm_fd_ = -1;
        return;
    }
}

SharedLidarReader::~SharedLidarReader() {
    if (shm_ptr_ && shm_ptr_ != MAP_FAILED) {
        size_t shm_size = sizeof(SharedLidar) - sizeof(pcl::PointXYZ) + 2 * max_points_ * sizeof(pcl::PointXYZ);
        munmap(shm_ptr_, shm_size);
    }
    if (shm_fd_ != -1) {
        close(shm_fd_);
    }
}

bool SharedLidarReader::isInitialized() const {
    return shm_ptr_ != nullptr && shm_ptr_ != MAP_FAILED;
}

bool SharedLidarReader::readPointCloud(pcl::PointCloud<pcl::PointXYZ> &cloud, int &last_frame) {
    if (!isInitialized()) return false;
    int idx = shm_ptr_->front_idx.load(std::memory_order_acquire);
    int frame = shm_ptr_->frame[idx];
    if (frame == last_frame) return false; // No new frame
    size_t num_points = shm_ptr_->num_points;
    cloud.clear();
    cloud.points.resize(num_points);
    memcpy(cloud.points.data(), shm_ptr_->data[idx], num_points * sizeof(pcl::PointXYZ));
    cloud.width = num_points;
    cloud.height = 1;
    cloud.is_dense = true;
    last_frame = frame;
    return true;
}

long SharedLidarReader::getTimeStamp() {
    int idx = shm_ptr_->front_idx.load(std::memory_order_acquire);
    return shm_ptr_->timeStamp[idx];
}

int SharedLidarReader::getFrameNumber() {
    int idx = shm_ptr_->front_idx.load(std::memory_order_acquire);
    return shm_ptr_->frame[idx];
}
