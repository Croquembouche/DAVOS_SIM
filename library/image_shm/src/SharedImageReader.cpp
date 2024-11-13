#include "SharedImage.h"
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <iostream>

SharedImageReader::SharedImageReader(const std::string &name)
    : shm_name_(name), shm_fd_(-1), shm_ptr_(nullptr) {
    // Open the shared memory segment
    shm_fd_ = shm_open(shm_name_.c_str(), O_RDONLY, 0666);
    if (shm_fd_ == -1) {
        std::cerr << "Failed to open shared memory for reading" << std::endl;
        return;
    }

    // Map the shared memory segment into the process's address space
    shm_ptr_ = static_cast<SharedImage *>(mmap(0, sizeof(SharedImage), PROT_READ, MAP_SHARED, shm_fd_, 0));

    if (shm_ptr_ == MAP_FAILED) {
        std::cerr << "Failed to map shared memory for reading" << std::endl;
    }
}

SharedImageReader::~SharedImageReader() {
    if (shm_ptr_ != MAP_FAILED) {
        munmap(shm_ptr_, sizeof(SharedImage)); // Unmap the shared memory
    }
    if (shm_fd_ != -1) {
        close(shm_fd_);                        // Close the shared memory file descriptor
    }
}

bool SharedImageReader::readImage(cv::Mat &image, int &last_version) {
    // Increment the reader count atomically before reading
    shm_ptr_->reader_count.fetch_add(1);

    // Check if a new version of the image is available
    if (shm_ptr_->version == last_version) {
        // Decrement reader count and notify the writer if no more readers are active
        if (shm_ptr_->reader_count.fetch_sub(1) == 1) {
            shm_ptr_->cv.notify_all();
        }
        return false; // No new data available
    }

    // Update the last_version to the current version
    last_version = shm_ptr_->version;

    // Copy shared memory data into an OpenCV Mat and clone to ensure thread safety
    image = cv::Mat(480, 640, CV_8UC3, shm_ptr_->data).clone();

    std::cout << "New image read with version " << last_version << std::endl;

    // Decrement reader count and notify the writer if no more readers are active
    if (shm_ptr_->reader_count.fetch_sub(1) == 1) {
        shm_ptr_->cv.notify_all(); // Notify writer that it can write
    }

    return true;
}


