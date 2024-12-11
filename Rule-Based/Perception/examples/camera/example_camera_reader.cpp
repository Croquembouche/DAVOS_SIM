#include "SharedImage.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <chrono>

int width = 640;
int height = 480;
int channels = 3;
char data_type[] = "CV_8U";

int main() {
    // Instantiate the shared memory reader
    char frontCamera[] = "front_camera";
    SharedImageReader reader(frontCamera, width, height, channels, data_type);

    // Check if shared memory mapping was successful
    if (!reader.isInitialized()) {
        std::cerr << "Failed to initialize shared memory reader. Exiting." << std::endl;
        return 1;  // Exit if shared memory setup failed
    }

    cv::Mat image;
    int last_frame = -1;

    auto start = std::chrono::high_resolution_clock::now();
    int frameCount = 0;
    int last_framenumber = 0;
    while (true) {
        // Attempt to read a new image from shared memory
        
        if (reader.readImage(image, last_frame)) {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            auto current = ts.tv_sec * 1e9 + ts.tv_nsec;
            auto writetime = reader.getTimeStamp();
            auto framenumber = reader.getFrameNumber();
            // std::cout << "Reader: New image read from shared memory (version " << last_frame << ")" << std::endl;
            if (framenumber-last_framenumber != 1){
                std::cout << "Missed: " << framenumber << std::endl;
            }
            last_framenumber = framenumber;
            std::cout << framenumber << ":" << current - writetime << std::endl;

            // Display the image
            // cv::imshow("Shared Memory Image", image);
            // if (cv::waitKey(1) == 27) {  // Press ESC to exit
            //     break;
            // }
            // frameCount++;
            // auto end = std::chrono::high_resolution_clock::now();
            // std::chrono::duration<double> elapsed = end - start;
            // int fps = 0;
            // if (elapsed.count() >= 1.0) {
            //     fps = frameCount / elapsed.count();
            //     std::cout << "FPS:" << fps << std::endl;
            //     frameCount = 0;
            //     start = std::chrono::high_resolution_clock::now();
            // }
            
        } 
        // else {
        //     std::cout << "Reader: No new image available in shared memory" << std::endl;
        // }

        // Check for updates every 100 ms
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return 0;
}


