#include "SharedImage.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <chrono>

#include <csignal>

// Global pointer to the writer for cleanup in the signal handler
SharedImageWriter* global_writer = nullptr;

// Signal handler for Ctrl+C
void handleSigInt(int signal) {
    if (global_writer) {
        std::cout << "Cleaning up shared memory..." << std::endl;
        delete global_writer; // Calls the destructor to clean up shared memory
        global_writer = nullptr;
    }
    std::cout << "Exiting program." << std::endl;
    std::exit(0); // Exit the program
}

int main() {
    // Instantiate the shared memory writer
    char frontCamera[] = "/front_camera";
    global_writer = new SharedImageWriter(frontCamera);

    // Register the signal handler
    std::signal(SIGINT, handleSigInt);

    // Create a sample image (640x480, RGB) with a red color
    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(0, 0, 255));  // A red image

    int iteration = 0;
    // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    while (true) {  // Write 10 times
        // Modify the image slightly each iteration (changing color)
        image.setTo(cv::Scalar(0, 0, 255-iteration*5));
        
        // Write the image to shared memory
        global_writer->writeImageCPU(image);
        // if (global_writer->writeImageCPU(image)) {

        //     std::cout << "Writer: Image written to shared memory (version " << iteration + 1 << ")" << std::endl;
        // } else {
        //     std::cerr << "Writer: Failed to write image to shared memory" << std::endl;
        // }

        // Wait for 1 second before writing the next image
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        iteration++;
    }

    return 0;
}


