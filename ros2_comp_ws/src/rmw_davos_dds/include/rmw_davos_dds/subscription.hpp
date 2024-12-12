#ifndef RMW_DAVOS_DDS_SUBSCRIPTION_HPP
#define RMW_DAVOS_DDS_SUBSCRIPTION_HPP

#include <rmw/rmw.h>
#include "rmw/error_handling.h"
#include "SharedImage.h"

extern "C" {
    rmw_subscription_t * rmw_create_subscription(
        const rmw_node_t * node,
        const rosidl_message_type_support_t * type_support,
        const char * topic_name,
        const rmw_qos_profile_t * qos_profile,
        const rmw_subscription_options_t * subscription_options);
    rmw_ret_t rmw_destroy_subscription(rmw_subscription_t * subscription);
    rmw_ret_t rmw_take(
        const rmw_subscription_t * subscription,
        void * ros_message,
        bool * taken,
        rmw_subscription_allocation_t * allocation);
    rmw_ret_t rmw_take_serialized_message(
        const rmw_subscription_t * subscription,
        rmw_serialized_message_t * serialized_message,
        bool * taken,
        rmw_subscription_allocation_t * allocation);
    rmw_ret_t rmw_subscription_get_actual_qos(
        const rmw_subscription_t * subscription,
        rmw_qos_profile_t * qos);
    rmw_ret_t rmw_subscription_set_content_filter(
        rmw_subscription_t * subscription,
        const rmw_subscription_content_filter_options_t * options);
    rmw_ret_t rmw_subscription_get_content_filter(
        const rmw_subscription_t * subscription,
        rcutils_allocator_t * allocator,
        rmw_subscription_content_filter_options_t * options);
}

#endif // RMW_DAVOS_DDS_SUBSCRIPTION_HPP
