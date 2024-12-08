#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include "SharedImage.h"

int main(int argc, char **argv) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create a node
  auto node = rclcpp::Node::make_shared("shared_memory_subscriber");

  // Initialize SharedImageReader for shared memory
  SharedImageReader reader("/image_topic");
//   if (!reader.isValid()) {
//     RCLCPP_ERROR(node->get_logger(), "Failed to initialize SharedImageReader");
//     return -1;
//   }

  // Timer-based subscription loop
  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(10),  // Poll every 10ms
    [&]() {
        cv::Mat image;
        int last_frame = -1;

        if (reader.readImageCPU(image, last_frame)) {
          RCLCPP_INFO(node->get_logger(), "Received frame with size: %dx%d", image.rows, image.cols);

          // Display the image (requires GUI support)
          cv::imshow("Received Image", image);
          cv::waitKey(1);
        } else {
          RCLCPP_ERROR(node->get_logger(), "Failed to read image from shared memory");
        }

    });

  // Spin the node to execute the timer
  rclcpp::spin(node);

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}
