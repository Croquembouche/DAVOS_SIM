#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <chrono>
#include "SharedImage.h"
#include <csignal>

// Global pointer to the writer for cleanup in the signal handler
SharedImageWriter* global_writer = nullptr;
int width = 640;
int height = 480;
int channels = 3;
char data_type[] = "CV_8U";

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
    // Open the camera using V4L2 backend for better performance
    cv::VideoCapture cap(cv::CAP_V4L2); // Access using Video4Linux2
    cap.open(0); // USB camera index 0
    if (!cap.isOpened()) {
        std::cerr << "Error: Unable to open the USB camera" << std::endl;
        return -1;
    }

    // Set camera resolution
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);

    // Check if GPU is available
    if (cv::cuda::getCudaEnabledDeviceCount() == 0) {
        std::cerr << "Error: No CUDA-compatible GPU found!" << std::endl;
        return -1;
    }

    // Set CUDA device
    cv::cuda::setDevice(0);

    cv::Mat frame;
    cv::cuda::GpuMat gpuFrame, resizedFrame;
    auto start = std::chrono::high_resolution_clock::now();
    int frameCount = 0;

    // Instantiate the shared memory writer
    char frontCamera[] = "front_camera";
    global_writer = new SharedImageWriter(frontCamera, height, width, channels, data_type);

    // Register the signal handler
    std::signal(SIGINT, handleSigInt);
    int iteration = 0;
    while (true) {
        // Capture frame from the camera 
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Error: Unable to grab a frame from the camera" << std::endl;
            break;
        }

        // Upload frame to GPU
        // gpuFrame.upload(frame);

        // // Resize the image to 640x480 using GPU
        // cv::cuda::resize(gpuFrame, resizedFrame, cv::Size(640, 480));

        // // Download the resized frame back to the CPU
        // cv::Mat resizedCpuFrame;
        // resizedFrame.download(resizedCpuFrame);

        // Calculate FPS
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
        // cv::imshow("Shared Memory Write Image", frame);
        //     if (cv::waitKey(100) == 27) {  // Press ESC to exit
        //         break;
        //     }
        // Write the image to shared memory
        if (global_writer->writeImage(frame)) {
            std::cout << "Writer: Image written to shared memory (version " << iteration + 1 << ")" << std::endl;
            iteration += 1;
        } else {
            std::cerr << "Writer: Failed to write image to shared memory" << std::endl;
        }
        
    }

    // Release the camera
    cap.release();
    // cv::destroyAllWindows();

    return 0;
}