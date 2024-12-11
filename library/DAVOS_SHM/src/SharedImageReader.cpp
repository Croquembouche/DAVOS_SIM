#include "SharedImage.h"
#include <cerrno>
#include <cstring>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>

SharedImageReader::SharedImageReader(const std::string &name, const int height, const int width, const int channels, const std::string &data_type)
    : shm_name_(name), shm_fd_(-1), shm_ptr_(nullptr) {
    shm_fd_ = shm_open(shm_name_.c_str(), O_RDWR, 0666);
    if (shm_fd_ == -1) {
        std::cerr << "Failed to open shared memory for reading. Error: " << strerror(errno) << std::endl;
        return;
    }

    int data_size = 1;
    if (data_type == "CV_8U"){
        data_size = 1;
    }else if (data_type == "CV_16U"){
        data_size = 2;
    }else if (data_type == "CV_32F"){
        data_size = 4;
    }else if (data_type == "CV_64F"){
        data_size = 8;
    }

    size_t shm_size_ = sizeof(SharedImage) + height*width*channels*data_size*sizeof(unsigned char);

    shm_ptr_ = static_cast<SharedImage *>(mmap(0, shm_size_, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0));
    if (shm_ptr_ == MAP_FAILED) {
        std::cerr << "Failed to map shared memory for reading. Error: " << strerror(errno) << std::endl;
        close(shm_fd_);
        shm_fd_ = -1;
        return;
    }
}

SharedImageReader::~SharedImageReader() {
    if (shm_ptr_ != MAP_FAILED) {
        munmap(shm_ptr_, sizeof(SharedImage) + shm_ptr_->size);
    }
    if (shm_fd_ != -1) {
        close(shm_fd_);
    }
}

bool SharedImageReader::isInitialized() const {
    return shm_ptr_ != nullptr;
}

bool SharedImageReader::readImage(cv::Mat &image, int &last_frame) {
    if (!isInitialized()) {
        std::cerr << "Shared memory not initialized." << std::endl;
        return false;
    }

    // if no new image, do not fetch image
    if (shm_ptr_->frame == last_frame){
        return false;
    }

    pthread_mutex_lock(&shm_ptr_->mutex);

    while (!shm_ptr_->ready_to_read) {
        shm_ptr_->waiting_readers++;
        pthread_cond_wait(&shm_ptr_->read_cv, &shm_ptr_->mutex);
        shm_ptr_->waiting_readers--;
    }

    if (shm_ptr_->frame > last_frame) {
        image = cv::Mat(480, 640, CV_8UC3, shm_ptr_->data);
        last_frame = shm_ptr_->frame;
    }

    pthread_mutex_unlock(&shm_ptr_->mutex);

    return true;
}

long SharedImageReader::getTimeStamp(){
    return shm_ptr_->timeStamp;
}

int SharedImageReader::getFrameNumber(){
    return shm_ptr_->frame;
}