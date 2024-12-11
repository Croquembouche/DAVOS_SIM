#include <cerrno>
#include <cstring>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <time.h>
#include <SharedUnitreeLidar.h>

SharedUnitreeLidarWriter::SharedUnitreeLidarWriter(const std::string &name, int cloud_scan_num)
    : shm_name_(name), shm_fd_(-1), shm_ptr_(nullptr) {
    shm_fd_ = shm_open(shm_name_.c_str(), O_CREAT | O_RDWR, 0666);
    if (shm_fd_ == -1) {
        std::cerr << "Failed to create shared memory. Error: " << strerror(errno) << std::endl;
        return;
    }

    size_t shm_size_ = sizeof(SharedUnitreeLidar)+(cloud_scan_num * 120 * sizeof(unsigned char));

    if (ftruncate(shm_fd_, shm_size_) == -1) {
        std::cerr << "Failed to set size of shared memory. Error: " << strerror(errno) << std::endl;
        close(shm_fd_);
        shm_fd_ = -1;
        return;
    }

    shm_ptr_ = static_cast<SharedUnitreeLidar *>(mmap(0, shm_size_, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0));
    if (shm_ptr_ == MAP_FAILED) {
        std::cerr << "Failed to map shared memory. Error: " << strerror(errno) << std::endl;
        close(shm_fd_);
        shm_fd_ = -1;
        return;
    }

    pthread_mutexattr_t mutex_attr;
    pthread_condattr_t cond_attr;

    pthread_mutexattr_init(&mutex_attr);
    pthread_mutexattr_setpshared(&mutex_attr, PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(&(shm_ptr_->mutex), &mutex_attr);
    pthread_mutexattr_destroy(&mutex_attr);

    pthread_condattr_init(&cond_attr);
    pthread_condattr_setpshared(&cond_attr, PTHREAD_PROCESS_SHARED);
    pthread_cond_init(&(shm_ptr_->read_cv), &cond_attr);
    pthread_condattr_destroy(&cond_attr);

    shm_ptr_->ready_to_read = false;
    shm_ptr_->waiting_readers = 0;
    shm_ptr_->frame = 0;
    shm_ptr_->size = cloud_scan_num * 120 * sizeof(unsigned char);
    std::cerr << "lidar data size:." << shm_ptr_->size << std::endl;
}

SharedUnitreeLidarWriter::~SharedUnitreeLidarWriter() {
    if (shm_ptr_ != MAP_FAILED) {
        pthread_mutex_destroy(&(shm_ptr_->mutex));
        pthread_cond_destroy(&(shm_ptr_->read_cv));
        munmap(shm_ptr_, sizeof(SharedUnitreeLidar)+shm_ptr_->size);
    }
    if (shm_fd_ != -1) {
        close(shm_fd_);
        shm_unlink(shm_name_.c_str());
    }
}


bool SharedUnitreeLidarWriter::writeLidar(const unitree_lidar_sdk::PointCloudUnitree &lidar) {
    if (!shm_ptr_) {
        std::cerr << "Shared memory is not initialized." << std::endl;
        return false;
    }
    if (lidar.points.size() > shm_ptr_->size) {
        std::cerr << "lidar data size too large." << shm_ptr_->size << std::endl;
        return false;
    }

    pthread_mutex_lock(&shm_ptr_->mutex);

    shm_ptr_->ready_to_read = false;
    memcpy(shm_ptr_->data, &lidar, lidar.points.size());
    shm_ptr_->frame += 1;

    shm_ptr_->ready_to_read = true;
    pthread_cond_broadcast(&shm_ptr_->read_cv);

    pthread_mutex_unlock(&shm_ptr_->mutex);
    return true;
}