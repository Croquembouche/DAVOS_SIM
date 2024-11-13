#include "SharedImage.h"
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

SharedImageWriter::SharedImageWriter(const std::string &name)
    : shm_name_(name), shm_fd_(-1), shm_ptr_(nullptr) {
    // Create or open the shared memory segment
    shm_fd_ = shm_open(shm_name_.c_str(), O_CREAT | O_RDWR, 0666);
    ftruncate(shm_fd_, sizeof(SharedImage));

    // Map the shared memory segment into the process's address space
    shm_ptr_ = static_cast<SharedImage *>(mmap(0, sizeof(SharedImage), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0));

    if (shm_ptr_ == MAP_FAILED) {
        std::cerr << "Failed to map shared memory for writing" << std::endl;
        return;
    }

    // Initialize reader count to 0
    shm_ptr_->reader_count = 0;
}

SharedImageWriter::~SharedImageWriter() {
    if (shm_ptr_ != MAP_FAILED) {
        munmap(shm_ptr_, sizeof(SharedImage)); // Unmap the shared memory
    }
    if (shm_fd_ != -1) {
        close(shm_fd_);                        // Close the shared memory file descriptor
        shm_unlink(shm_name_.c_str());         // Unlink the shared memory segment
    }
}

bool SharedImageWriter::writeImage(const cv::Mat &image) {
    if (image.empty() || image.total() * image.elemSize() != 921600) {
        std::cerr << "Image size mismatch." << std::endl;
        return false;
    }

    // Lock the mutex associated with the condition variable
    std::unique_lock<std::mutex> lock(shm_ptr_->cv_mutex);

    // Wait until there are no active readers
    shm_ptr_->cv.wait(lock, [&] { return shm_ptr_->reader_count.load() == 0; });

    // Perform the write operation
    memcpy(shm_ptr_->data, image.data, 921600);  // Copy image data to shared memory
    shm_ptr_->version += 1;                      // Increment version to signal new data

    std::cout << "Image written to shared memory with version " << shm_ptr_->version << std::endl;

    // Notify all readers that new data is available
    shm_ptr_->cv.notify_all();

    return true;
}


