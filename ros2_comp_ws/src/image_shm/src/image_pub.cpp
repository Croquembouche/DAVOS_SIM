#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class ImagePublisher : public rclcpp::Node
{
public:
    ImagePublisher()        
    : Node("image_publisher"), frame_count_(0) // Initialize frame count to 0

    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_topic", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ImagePublisher::publish_image, this));
    }

    int frame_id = 1;

private:
    void publish_image()
    {
        // Create an image message
        auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
        
        image_msg->header.frame_id = std::to_string(frame_count_); // Set frame_id as the frame count

        image_msg->height = 480;
        image_msg->width = 640;
        image_msg->encoding = "rgb8"; // Red-Green-Blue format
        image_msg->is_bigendian = false;
        image_msg->step = image_msg->width * 3;

        // Fill the image data with red pixels
        image_msg->data.resize(image_msg->step * image_msg->height);
        for (size_t i = 0; i < image_msg->data.size(); i += 3)
        {
            image_msg->data[i] = 255;     // Red channel
            image_msg->data[i + 1] = 0;  // Green channel
            image_msg->data[i + 2] = 0;  // Blue channel
        }
        image_msg->header.stamp = this->now();
        // Publish the image
        publisher_->publish(*image_msg);
        // RCLCPP_INFO(this->get_logger(), "Published an image at: %ld ns", image_msg->header.stamp.nanosec());
        RCLCPP_INFO(this->get_logger(), "Published an image at: %d ", frame_count_);

        // Increment the frame count
        frame_count_++;
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int frame_count_; // Frame counter
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisher>());
    rclcpp::shutdown();
    return 0;
}
