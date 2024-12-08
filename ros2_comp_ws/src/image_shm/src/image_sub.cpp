#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber()
        : Node("image_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_topic", 10,
            std::bind(&ImageSubscriber::image_callback, this, std::placeholders::_1));
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Calculate the latency
        auto receive_time = this->now();
        auto publish_time = msg->header.stamp;
        auto latency_ns = (receive_time - publish_time).nanoseconds();
        auto frame_count_ = msg->header.frame_id;

        RCLCPP_INFO(this->get_logger(), "%s at: %ld", frame_count_.c_str(), latency_ns);
        //if (std::stoi(frame_count_)-previous != 1){
          //  RCLCPP_INFO(this->get_logger(), "Lost frame: %s", frame_count_.c_str());
        //}

        previous = std::stoi(frame_count_);

        // Optional: Print the first pixel values
        // if (!msg->data.empty())
        // {
        //     RCLCPP_INFO(this->get_logger(), "First pixel: R=%d, G=%d, B=%d",
        //                 msg->data[0], msg->data[1], msg->data[2]);
        // }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    int previous = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSubscriber>());
    rclcpp::shutdown();
    return 0;
}
