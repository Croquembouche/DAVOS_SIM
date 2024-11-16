#include "SharedImage.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <chrono>


int main() {
    // Instantiate the shared memory reader
    char frontCamera[] = "/front_camera";
    SharedImageReader reader(frontCamera);

    // Check if shared memory mapping was successful
    if (!reader.isInitialized()) {
        std::cerr << "Failed to initialize shared memory reader. Exiting." << std::endl;
        return 1;  // Exit if shared memory setup failed
    }

    cv::Mat image;
    int last_frame = -1;

    while (true) {
        // Attempt to read a new image from shared memory
        
        if (reader.readImage(image, last_frame)) {
            std::cout << "Reader: New image read from shared memory (version " << last_frame << ")" << std::endl;

            // Display the image
            // cv::imshow("Shared Memory Image", image);
            // if (cv::waitKey(100) == 27) {  // Press ESC to exit
            //     break;
            // }
        } 
        // else {
        //     std::cout << "Reader: No new image available in shared memory" << std::endl;
        // }

        // Check for updates every 100 ms
        // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return 0;
}


