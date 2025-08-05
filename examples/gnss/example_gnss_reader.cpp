#include "SharedGNSS.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <ctime>

int main() {
    SharedGNSSReader* gnss_reader_ = new SharedGNSSReader("sensing_gnss_pose_data");

    if (!gnss_reader_->isInitialized()) {
        std::cerr << "GNSS Reader initialization failed!" << std::endl;
        delete gnss_reader_;
        return 1;
    }
    
    std::cout << "GNSS Reader initialized successfully!" << std::endl;

    geometry_msgs::msg::PoseWithCovarianceStamped gnss_data;
    int last_frame = -1;
    int last_framenumber = 0;

    while (true) {
        if (gnss_reader_->readGNSS(gnss_data, last_frame)) {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            auto current = ts.tv_sec * 1e9 + ts.tv_nsec;
            auto writetime = gnss_reader_->getTimeStamp();
            auto framenumber = gnss_reader_->getFrameNumber();

            if (framenumber - last_framenumber != 1) {
                std::cout << "Missed frames: " << framenumber << ";" << last_framenumber << std::endl;
            }
            last_framenumber = framenumber;

            // Display the data from the received message
            std::cout << "Frame: " << framenumber 
                      << ", Latency: " << (current - writetime) / 1000000 << " ms" 
                      << ", Frame ID: " << gnss_data.header.frame_id
                      << "\nPosition - X: " << gnss_data.pose.pose.position.x
                      << ", Y: " << gnss_data.pose.pose.position.y 
                      << ", Z: " << gnss_data.pose.pose.position.z
                      << "\nOrientation - X: " << gnss_data.pose.pose.orientation.x
                      << ", Y: " << gnss_data.pose.pose.orientation.y
                      << ", Z: " << gnss_data.pose.pose.orientation.z
                      << ", W: " << gnss_data.pose.pose.orientation.w
                      << std::endl;
        }
        // std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 10 ms sleep
    }

    delete gnss_reader_;
    return 0;
}
