#include "SharedUnitreeLidar.h"
#include <cerrno>
#include <cstring>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>

SharedUnitreeLidarReader::SharedUnitreeLidarReader(const std::string &name, int cloud_scan_num, int point_byte)
    : shm_name_(name), shm_fd_(-1), shm_ptr_(nullptr) {
    shm_fd_ = shm_open(shm_name_.c_str(), O_RDWR, 0666);
    if (shm_fd_ == -1) {
        std::cerr << "Failed to open shared memory for reading. Error: " << strerror(errno) << std::endl;
        return;
    }

    size_t shm_size_ = sizeof(SharedUnitreeLidar)+(cloud_scan_num * 120 * point_byte);

    shm_ptr_ = static_cast<SharedUnitreeLidar *>(mmap(0, shm_size_, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0));
    if (shm_ptr_ == MAP_FAILED) {
        std::cerr << "Failed to map shared memory for reading. Error: " << strerror(errno) << std::endl;
        close(shm_fd_);
        shm_fd_ = -1;
        return;
    }
    std::cerr << "Shared memory is Found." << std::endl;
}

SharedUnitreeLidarReader::~SharedUnitreeLidarReader() {
    if (shm_ptr_ != MAP_FAILED) {
        munmap(shm_ptr_, sizeof(SharedUnitreeLidar) + shm_ptr_->size);
        // std::cerr << "Shared memory freed." << std::endl;
    }
    if (shm_fd_ != -1) {
        close(shm_fd_);
    }
}

bool SharedUnitreeLidarReader::isInitialized() const {
    return shm_ptr_ != nullptr;
}

bool SharedUnitreeLidarReader::readLidar(unitree_lidar_sdk::PointCloudUnitree &lidar, int &last_frame) {
    if (!isInitialized()) {
        std::cerr << "Shared memory not initialized." << std::endl;
        return false;
    }

    // Check if the frame has changed, and if not, return false to indicate no new data
    if (shm_ptr_->frame == last_frame) {
        return false;
    }

    pthread_mutex_lock(&shm_ptr_->mutex);
    // Wait until the data is ready to be read
    while (!shm_ptr_->ready_to_read) {
        shm_ptr_->waiting_readers++;
        pthread_cond_wait(&shm_ptr_->read_cv, &shm_ptr_->mutex);
        shm_ptr_->waiting_readers--;
    }

    // If a new frame is available, copy the lidar data into the lidar object
    if (shm_ptr_->frame > last_frame) {
        // Calculate the number of points from the size of the data in shared memory
        size_t num_points = shm_ptr_->size / sizeof(lidar.points[0]);

        // Resize the lidar point cloud to match the number of points
        lidar.points.resize(num_points);

        // Copy the point data from shared memory to lidar.points
        memcpy(lidar.points.data(), shm_ptr_->data, shm_ptr_->size);

        last_frame = shm_ptr_->frame;
    }

    pthread_mutex_unlock(&shm_ptr_->mutex);

    return true;
}

long SharedUnitreeLidarReader::getTimeStamp(){
    return shm_ptr_->timeStamp;
}


int SharedUnitreeLidarReader::getFrameNumber(){
    return shm_ptr_->frame;
}