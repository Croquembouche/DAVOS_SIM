#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

class PointCloudSubscriber : public rclcpp::Node
{
public:
    PointCloudSubscriber()
        : Node("point_cloud_subscriber")
    {
        // Create a subscription to /unilidar/cloud topic
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/unilidar/cloud", 10,
            std::bind(&PointCloudSubscriber::listener_callback, this, std::placeholders::_1));
    }

private:
    void listener_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Extract the sent timestamp from the header
        rclcpp::Time sent_timestamp(msg->header.stamp);

        // Get the current time (receive timestamp)
        rclcpp::Time receive_timestamp = this->get_clock()->now();

        // Calculate the time difference (latency) in milliseconds
        rclcpp::Duration duration = receive_timestamp - sent_timestamp;

        // Convert the duration to milliseconds
        auto latency_ns = duration.nanoseconds();  // nanoseconds to milliseconds

        RCLCPP_INFO(this->get_logger(), "%ld", latency_ns);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudSubscriber>());
    rclcpp::shutdown();
    return 0;
}
