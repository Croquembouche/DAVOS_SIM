#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include "davos_dds_example/type_adapter.hpp"  // Include your custom type support header

class CustomMatPublisherNode : public rclcpp::Node {
public:
    CustomMatPublisherNode()
        : Node("custom_mat_publisher_node") {
        // Create the publisher for cv::Mat
        publisher_ = this->create_publisher<cv::Mat>("front_camera", 10);

        // Set up the timer to periodically publish an image
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&CustomMatPublisherNode::publish_image, this));
    }

private:
    void publish_image() {
        // Create a dummy OpenCV image
        cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);  // Black image
        for (int y = 0; y < image.rows; ++y) {
            for (int x = 0; x < image.cols; ++x) {
                image.at<cv::Vec3b>(y, x) = cv::Vec3b(x % 256, y % 256, (x + y) % 256);
            }
        }

        // Publish the cv::Mat image
        publisher_->publish(image);

        // Log the publication
        RCLCPP_INFO(this->get_logger(), "Published a cv::Mat image.");
    }

    rclcpp::Publisher<cv::Mat>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    // Set the RMW implementation to rmw_davos_dds explicitly
    if (setenv("RMW_IMPLEMENTATION", "rmw_davos_dds", 1) != 0) {
        fprintf(stderr, "Failed to set RMW_IMPLEMENTATION environment variable\n");
        return 1;
    }

    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Spin the node
    rclcpp::spin(std::make_shared<CustomMatPublisherNode>());

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
