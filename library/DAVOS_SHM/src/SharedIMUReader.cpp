#include "SharedIMU.h"
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <errno.h>
#include <sys/stat.h>

SharedIMUReader::SharedIMUReader(const std::string &name)
    : shm_name_(name), shm_fd_(-1), shm_ptr_(nullptr) {
    size_t shm_size = sizeof(SharedIMU);

    shm_fd_ = shm_open(shm_name_.c_str(), O_RDONLY, 0666);
    if (shm_fd_ == -1) {
        std::cerr << "Failed to open shared memory. Error: " << strerror(errno) << std::endl;
        return;
    }

    shm_ptr_ = static_cast<SharedIMU*>(mmap(0, shm_size, PROT_READ, MAP_SHARED, shm_fd_, 0));
    if (shm_ptr_ == MAP_FAILED) {
        std::cerr << "Failed to map shared memory. Error: " << strerror(errno) << std::endl;
        close(shm_fd_);
        shm_fd_ = -1;
        return;
    }
}

SharedIMUReader::~SharedIMUReader() {
    if (shm_ptr_ && shm_ptr_ != MAP_FAILED) {
        size_t shm_size = sizeof(SharedIMU);
        munmap(shm_ptr_, shm_size);
    }
    if (shm_fd_ != -1) {
        close(shm_fd_);
    }
}

bool SharedIMUReader::readIMU(SharedIMU::IMUData &imuData, int &last_frame) {
    if (!shm_ptr_) return false;
    
    int front_idx = shm_ptr_->front_idx.load(std::memory_order_acquire);

    // Check if the frame has already been read
    if (shm_ptr_->frame[front_idx] == last_frame) {
        return false; // No new data
    }

    // Copy the data directly from shared memory to the output structure
    imuData.frame_id_length = shm_ptr_->imu_data[front_idx].frame_id_length;
    std::strncpy(imuData.frame_id, 
                shm_ptr_->imu_data[front_idx].frame_id, 
                imuData.frame_id_length);
    imuData.frame_id[imuData.frame_id_length] = '\0'; // Ensure null termination
    
    // Copy twist data
    imuData.linear_x = shm_ptr_->imu_data[front_idx].linear_x;
    imuData.linear_y = shm_ptr_->imu_data[front_idx].linear_y;
    imuData.linear_z = shm_ptr_->imu_data[front_idx].linear_z;
    imuData.angular_x = shm_ptr_->imu_data[front_idx].angular_x;
    imuData.angular_y = shm_ptr_->imu_data[front_idx].angular_y;
    imuData.angular_z = shm_ptr_->imu_data[front_idx].angular_z;
    
    // Copy covariance data
    for (size_t i = 0; i < 36; ++i) {
        imuData.covariance[i] = shm_ptr_->imu_data[front_idx].covariance[i];
    }

    last_frame = shm_ptr_->frame[front_idx];
    return true;
}

bool SharedIMUReader::isInitialized() const {
    return shm_ptr_ != nullptr && shm_ptr_ != MAP_FAILED;
}

long SharedIMUReader::getTimeStamp() {
    if (!shm_ptr_) return -1;

    int front_idx = shm_ptr_->front_idx.load(std::memory_order_acquire);
    return shm_ptr_->timeStamp[front_idx];
}

int SharedIMUReader::getFrameNumber() {
    if (!shm_ptr_) return -1;

    int front_idx = shm_ptr_->front_idx.load(std::memory_order_acquire);
    return shm_ptr_->frame[front_idx];
}
