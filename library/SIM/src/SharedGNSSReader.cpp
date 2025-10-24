#include "SharedGNSS.h"
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <errno.h>
#include <sys/stat.h>

SharedGNSSReader::SharedGNSSReader(const std::string &name)
    : shm_name_(name), shm_fd_(-1), shm_ptr_(nullptr) {
    size_t shm_size = sizeof(SharedGNSS);

    shm_fd_ = shm_open(shm_name_.c_str(), O_RDONLY, 0666);
    if (shm_fd_ == -1) {
        std::cerr << "Failed to open shared memory. Error: " << strerror(errno) << std::endl;
        return;
    }

    shm_ptr_ = static_cast<SharedGNSS*>(mmap(0, shm_size, PROT_READ, MAP_SHARED, shm_fd_, 0));
    if (shm_ptr_ == MAP_FAILED) {
        std::cerr << "Failed to map shared memory. Error: " << strerror(errno) << std::endl;
        close(shm_fd_);
        shm_fd_ = -1;
        return;
    }
}

SharedGNSSReader::~SharedGNSSReader() {
    if (shm_ptr_ && shm_ptr_ != MAP_FAILED) {
        size_t shm_size = sizeof(SharedGNSS);
        munmap(shm_ptr_, shm_size);
    }
    if (shm_fd_ != -1) {
        close(shm_fd_);
    }
}

bool SharedGNSSReader::readGNSS(geometry_msgs::msg::PoseWithCovarianceStamped &gnssMsg, int &last_frame) {
    if (!shm_ptr_) return false;
    
    int front_idx = shm_ptr_->front_idx.load(std::memory_order_acquire);

    // Check if the frame has already been read
    if (shm_ptr_->frame[front_idx] == last_frame) {
        return false; // No new data
    }

    // Read the GNSS data from the custom structure
    // Set header frame_id
    std::string frame_id(shm_ptr_->gnss_data[front_idx].frame_id, 
                        shm_ptr_->gnss_data[front_idx].frame_id_length);
    gnssMsg.header.frame_id = frame_id;
    
    // Set pose data
    gnssMsg.pose.pose.position.x = shm_ptr_->gnss_data[front_idx].position_x;
    gnssMsg.pose.pose.position.y = shm_ptr_->gnss_data[front_idx].position_y;
    gnssMsg.pose.pose.position.z = shm_ptr_->gnss_data[front_idx].position_z;
    
    gnssMsg.pose.pose.orientation.x = shm_ptr_->gnss_data[front_idx].orientation_x;
    gnssMsg.pose.pose.orientation.y = shm_ptr_->gnss_data[front_idx].orientation_y;
    gnssMsg.pose.pose.orientation.z = shm_ptr_->gnss_data[front_idx].orientation_z;
    gnssMsg.pose.pose.orientation.w = shm_ptr_->gnss_data[front_idx].orientation_w;
    
    // Copy covariance data
    for (size_t i = 0; i < 36; ++i) {
        gnssMsg.pose.covariance[i] = shm_ptr_->gnss_data[front_idx].covariance[i];
    }

    last_frame = shm_ptr_->frame[front_idx];
    return true;
}

bool SharedGNSSReader::isInitialized() const {
    return shm_ptr_ != nullptr && shm_ptr_ != MAP_FAILED;
}

long SharedGNSSReader::getTimeStamp() {
    if (!shm_ptr_) return -1;

    int front_idx = shm_ptr_->front_idx.load(std::memory_order_acquire);
    return shm_ptr_->timeStamp[front_idx];
}

int SharedGNSSReader::getFrameNumber() {
    if (!shm_ptr_) return -1;

    int front_idx = shm_ptr_->front_idx.load(std::memory_order_acquire);
    return shm_ptr_->frame[front_idx];
}
