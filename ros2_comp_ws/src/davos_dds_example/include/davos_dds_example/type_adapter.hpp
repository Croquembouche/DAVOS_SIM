#ifndef TYPE_SUPPORT_CV_MAT_HPP
#define TYPE_SUPPORT_CV_MAT_HPP

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosidl_runtime_cpp/traits.hpp>
#include <rosidl_runtime_c/message_type_support_struct.h>

// Fake type support for cv::Mat
static const rosidl_message_type_support_t fake_mat_type_support = {
    "rmw_davos_dds_identifier",  // Replace with your RMW's identifier
    nullptr,                     // Serialization callbacks (not needed)
    nullptr                      // Introspection callbacks (not needed)
};

// Specialize rosidl_generator_traits::is_message for cv::Mat
namespace rosidl_generator_traits {

template <>
struct is_message<cv::Mat> : std::true_type {};

}  // namespace rosidl_generator_traits

// Specialize rclcpp::TypeAdapter for cv::Mat
namespace rclcpp {

template <>
struct TypeAdapter<cv::Mat> {
    using is_specialized = std::false_type;  // No type adaptation
    using custom_type = cv::Mat;
    using ros_message_type = cv::Mat;
};

}  // namespace rclcpp

// Provide fake type support handle
namespace rosidl_typesupport_cpp {

template <>
const rosidl_message_type_support_t * get_message_type_support_handle<cv::Mat>() {
    return &fake_mat_type_support;  // Return the fake type support
}

}  // namespace rosidl_typesupport_cpp

#endif  // TYPE_SUPPORT_CV_MAT_HPP
