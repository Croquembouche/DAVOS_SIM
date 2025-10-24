#include "SharedGNSS.h"
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <errno.h>
#include <sys/stat.h>
#include <time.h>

SharedGNSSWriter::SharedGNSSWriter(const std::string &name)
    : shm_name_(name), shm_fd_(-1), shm_ptr_(nullptr) {
    size_t shm_size = sizeof(SharedGNSS);

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
    shm_ptr_ = static_cast<SharedGNSS*>(mmap(0, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0));
    if (shm_ptr_ == MAP_FAILED) {
        std::cerr << "Failed to map shared memory. Error: " << strerror(errno) << std::endl;
        close(shm_fd_);
        shm_fd_ = -1;
        return;
    }

    shm_ptr_->front_idx.store(0, std::memory_order_relaxed);
    shm_ptr_->frame[0] = shm_ptr_->frame[1] = 0;
    shm_ptr_->timeStamp[0] = shm_ptr_->timeStamp[1] = 0;
    std::cerr << "GNSS Writer Init Success" << std::endl;
}

SharedGNSSWriter::~SharedGNSSWriter() {
    if (shm_ptr_ && shm_ptr_ != MAP_FAILED) {
        size_t shm_size = sizeof(SharedGNSS);
        munmap(shm_ptr_, shm_size);
    }
    if (shm_fd_ != -1) {
        close(shm_fd_);
        shm_unlink(shm_name_.c_str());
    }
}

bool SharedGNSSWriter::writeGNSS(const geometry_msgs::msg::PoseWithCovarianceStamped &gnssMsg) {
    if (!shm_ptr_) return false;
    int next_idx = 1 - shm_ptr_->front_idx.load(std::memory_order_acquire);
    
    // Copy frame_id safely
    size_t frame_id_length = gnssMsg.header.frame_id.size();
    if (frame_id_length > 255) frame_id_length = 255; // Ensure it fits in our buffer
    shm_ptr_->gnss_data[next_idx].frame_id_length = frame_id_length;
    std::strncpy(shm_ptr_->gnss_data[next_idx].frame_id, 
                gnssMsg.header.frame_id.c_str(), frame_id_length);
    shm_ptr_->gnss_data[next_idx].frame_id[frame_id_length] = '\0'; // Null terminate
    
    // Copy pose data
    shm_ptr_->gnss_data[next_idx].position_x = gnssMsg.pose.pose.position.x;
    shm_ptr_->gnss_data[next_idx].position_y = gnssMsg.pose.pose.position.y;
    shm_ptr_->gnss_data[next_idx].position_z = gnssMsg.pose.pose.position.z;
    
    shm_ptr_->gnss_data[next_idx].orientation_x = gnssMsg.pose.pose.orientation.x;
    shm_ptr_->gnss_data[next_idx].orientation_y = gnssMsg.pose.pose.orientation.y;
    shm_ptr_->gnss_data[next_idx].orientation_z = gnssMsg.pose.pose.orientation.z;
    shm_ptr_->gnss_data[next_idx].orientation_w = gnssMsg.pose.pose.orientation.w;
    
    // Copy covariance data
    for (size_t i = 0; i < 36; ++i) {
        shm_ptr_->gnss_data[next_idx].covariance[i] = gnssMsg.pose.covariance[i];
    }
    
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    shm_ptr_->timeStamp[next_idx] = ts.tv_sec * 1000000000L + ts.tv_nsec;
    shm_ptr_->frame[next_idx] = shm_ptr_->frame[1 - next_idx] + 1;

    // Atomically publish the new buffer
    shm_ptr_->front_idx.store(next_idx, std::memory_order_release);
    return true;
}
