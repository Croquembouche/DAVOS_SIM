#include "SharedIMU.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <ctime>

int main() {
    SharedIMUReader* imu_reader_ = new SharedIMUReader("sensing_imu_imu_data");

    if (!imu_reader_->isInitialized()) {
        std::cerr << "Reader initialization failed!" << std::endl;
        delete imu_reader_;
        return 1;
    }
    
    std::cout << "Reader initialized successfully!" << std::endl;

    SharedIMU::IMUData imu_data;
    int last_frame = -1;
    int last_framenumber = 0;

    while (true) {
        if (imu_reader_->readIMU(imu_data, last_frame)) {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            auto current = ts.tv_sec * 1e9 + ts.tv_nsec;
            auto writetime = imu_reader_->getTimeStamp();
            auto framenumber = imu_reader_->getFrameNumber();

            if (framenumber - last_framenumber != 1) {
                std::cout << "Missed: " << framenumber << ";" << last_framenumber << std::endl;
            }
            last_framenumber = framenumber;

            // Display the data from the received struct
            std::cout << "Frame: " << framenumber 
                      << ", Latency: " << (current - writetime) / 1000000 << " ms" 
                      << ", Frame ID: " << std::string(imu_data.frame_id, imu_data.frame_id_length)
                      << "\nLinear velocity - X: " << imu_data.linear_x
                      << ", Y: " << imu_data.linear_y
                      << ", Z: " << imu_data.linear_z
                      << "\nAngular velocity - X: " << imu_data.angular_x
                      << ", Y: " << imu_data.angular_y
                      << ", Z: " << imu_data.angular_z
                      << std::endl;
        }
        // std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 10 ms sleep
    }

    delete imu_reader_;
    return 0;
}
