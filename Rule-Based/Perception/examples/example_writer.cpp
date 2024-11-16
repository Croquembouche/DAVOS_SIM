#include "SharedImage.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    // Instantiate the shared memory writer
    char frontCamera[] = "/front_camera";
    SharedImageWriter writer(frontCamera);

    // Create a sample image (640x480, RGB) with a red color
    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(0, 0, 255));  // A red image

    int iteration = 0;
    while (true) {  // Write 10 times
        // Modify the image slightly each iteration (changing color)
        image.setTo(cv::Scalar(0, 0, 255-iteration*5));
        
        // Write the image to shared memory
        if (writer.writeImage(image)) {
            // std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
            // auto duration = now.time_since_epoch();
            // auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
            std::cout << "Writer: Image written to shared memory (version " << iteration + 1 << ")" << std::endl;
            // cv::imshow("Shared Memory Image", image);
            // if (cv::waitKey(100) == 27) {  // Press ESC to exit
            //     break;
            // }
        } else {
            std::cerr << "Writer: Failed to write image to shared memory" << std::endl;
        }

        // Wait for 1 second before writing the next image
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        iteration++;
    }

    return 0;
}


