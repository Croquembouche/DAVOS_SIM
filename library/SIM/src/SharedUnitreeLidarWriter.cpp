#include <cerrno>
#include <cstring>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <time.h>
#include <SharedUnitreeLidar.h>

SharedUnitreeLidarWriter::SharedUnitreeLidarWriter(const std::string &name, int cloud_scan_num, int point_byte)
    : shm_name_(name), shm_fd_(-1), shm_ptr_(nullptr) {
    // Open or create shared memory
    shm_fd_ = shm_open(shm_name_.c_str(), O_CREAT | O_RDWR, 0666);
    if (shm_fd_ == -1) {
        std::cerr << "Failed to create shared memory. Error: " << strerror(errno) << std::endl;
        return;
    }

    // Calculate the total size required for the shared memory
    size_t shm_size_ = sizeof(SharedUnitreeLidar) + (cloud_scan_num * 120 * point_byte);
    
    // Set the size of the shared memory region
    if (ftruncate(shm_fd_, shm_size_) == -1) {
        std::cerr << "Failed to set size of shared memory. Error: " << strerror(errno) << std::endl;
        close(shm_fd_);
        shm_fd_ = -1;
        return;
    }

    // Map the shared memory into the process's address space
    shm_ptr_ = static_cast<SharedUnitreeLidar *>(mmap(0, shm_size_, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0));
    if (shm_ptr_ == MAP_FAILED) {
        std::cerr << "Failed to map shared memory. Error: " << strerror(errno) << std::endl;
        close(shm_fd_);
        shm_fd_ = -1;
        return;
    }

    // Initialize mutex and condition variable for synchronization
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

    // Initialize shared memory structure fields
    shm_ptr_->ready_to_read = false;
    shm_ptr_->waiting_readers = 0;
    shm_ptr_->frame = 0;
    shm_ptr_->size = cloud_scan_num * 120 * point_byte;
}

SharedUnitreeLidarWriter::~SharedUnitreeLidarWriter() {
    if (shm_ptr_ != MAP_FAILED) {
        pthread_mutex_destroy(&(shm_ptr_->mutex));
        pthread_cond_destroy(&(shm_ptr_->read_cv));
        munmap(shm_ptr_, sizeof(SharedUnitreeLidar) + shm_ptr_->size); // Unmap the shared memory
    }
    if (shm_fd_ != -1) {
        close(shm_fd_);
        shm_unlink(shm_name_.c_str()); // Unlink the shared memory object
    }
}

bool SharedUnitreeLidarWriter::writeLidar(unitree_lidar_sdk::PointCloudUnitree &lidar) {
    if (!shm_ptr_) {
        std::cerr << "Shared memory is not initialized." << std::endl;
        return false;
    }

    // Ensure lidar data fits in the available shared memory space
    if (lidar.points.size() * sizeof(lidar.points[0]) > shm_ptr_->size) {
        std::cerr << "Lidar data size too large: " << lidar.points.size() * sizeof(lidar.points[0]) << " bytes, available: " << shm_ptr_->size << " bytes." << std::endl;
        return false;
    }

    pthread_mutex_lock(&shm_ptr_->mutex);

    shm_ptr_->ready_to_read = false;

    // Copy the point cloud data into shared memory (copy only the points data)
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    memcpy(shm_ptr_->data, lidar.points.data(), lidar.points.size() * sizeof(lidar.points[0]));
    shm_ptr_->timeStamp = ts.tv_sec * 1e9 + ts.tv_nsec;
    shm_ptr_->frame += 1;
    shm_ptr_->ready_to_read = true;

    // Notify readers that data is available
    pthread_cond_broadcast(&shm_ptr_->read_cv);

    pthread_mutex_unlock(&shm_ptr_->mutex);
    return true;
}
