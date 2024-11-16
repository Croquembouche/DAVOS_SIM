#include "SharedImage.h"
#include <cerrno>
#include <cstring>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>

SharedImageWriter::SharedImageWriter(const std::string &name)
    : shm_name_(name), shm_fd_(-1), shm_ptr_(nullptr) {
    shm_fd_ = shm_open(shm_name_.c_str(), O_CREAT | O_RDWR, 0666);
    if (shm_fd_ == -1) {
        std::cerr << "Failed to create shared memory. Error: " << strerror(errno) << std::endl;
        return;
    }

    if (ftruncate(shm_fd_, sizeof(SharedImage)) == -1) {
        std::cerr << "Failed to set size of shared memory. Error: " << strerror(errno) << std::endl;
        close(shm_fd_);
        shm_fd_ = -1;
        return;
    }

    shm_ptr_ = static_cast<SharedImage *>(mmap(0, sizeof(SharedImage), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0));
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
}

SharedImageWriter::~SharedImageWriter() {
    if (shm_ptr_ != MAP_FAILED) {
        pthread_mutex_destroy(&(shm_ptr_->mutex));
        pthread_cond_destroy(&(shm_ptr_->read_cv));
        munmap(shm_ptr_, sizeof(SharedImage));
    }
    if (shm_fd_ != -1) {
        close(shm_fd_);
        shm_unlink(shm_name_.c_str());
    }
}

int getSystemUptimeAsInt() {
    struct timespec ts;
    if (clock_gettime(CLOCK_BOOTTIME, &ts) == -1) {
        perror("clock_gettime");
        return -1;
    }

    // Convert seconds and nanoseconds to milliseconds
    return static_cast<int>(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

bool SharedImageWriter::writeImage(const cv::Mat &image, int &timeStamp) {
    if (image.empty() || image.total() * image.elemSize() != 921600) {
        std::cerr << "Image size mismatch." << std::endl;
        return false;
    }

    pthread_mutex_lock(&shm_ptr_->mutex);

    shm_ptr_->ready_to_read = false;

    memcpy(shm_ptr_->data, image.data, 921600);
    shm_ptr_->frame += 1;
    timeStamp = getSystemUptimeAsInt();
    shm_ptr_->timeStamp = timeStamp;

    shm_ptr_->ready_to_read = true;
    pthread_cond_broadcast(&shm_ptr_->read_cv);

    pthread_mutex_unlock(&shm_ptr_->mutex);

    return true;
}
