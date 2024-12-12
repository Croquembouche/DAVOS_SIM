#include "rmw_davos_dds/subscription.hpp"

rmw_subscription_t * rmw_create_subscription(
    const rmw_node_t * node,
    const rosidl_message_type_support_t * type_support,
    const char * topic_name,
    const rmw_qos_profile_t * qos_profile,
    const rmw_subscription_options_t * subscription_options) 
{
    // Validate arguments
    if (!node || !type_support || !topic_name) {
        RMW_SET_ERROR_MSG("Invalid arguments in rmw_create_subscription");
        return nullptr;
    }

    // Allocate memory for the subscription
    auto * subscription = new rmw_subscription_t;
    subscription->topic_name = strdup(topic_name);

    if (!subscription->topic_name) {
        RMW_SET_ERROR_MSG("Failed to allocate memory for topic name");
        delete subscription;
        return nullptr;
    }

    try {
        // Create the shared memory reader for this subscription
        SharedImageReader * reader = new SharedImageReader(topic_name);

        // Associate the reader with the subscription
        subscription->data = reader;
        return subscription;

    } catch (const std::exception & e) {
        // Clean up in case of an error
        RMW_SET_ERROR_MSG(e.what());
        delete subscription;
        return nullptr;
    }
}

rmw_ret_t rmw_destroy_subscription(rmw_subscription_t * subscription) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_take(
    const rmw_subscription_t * subscription,
    void * ros_message,
    bool * taken,
    rmw_subscription_allocation_t * allocation) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_take_serialized_message(
    const rmw_subscription_t * subscription,
    rmw_serialized_message_t * serialized_message,
    bool * taken,
    rmw_subscription_allocation_t * allocation) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_subscription_get_actual_qos(
    const rmw_subscription_t * subscription,
    rmw_qos_profile_t * qos) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_subscription_set_content_filter(
    rmw_subscription_t * subscription,
    const rmw_subscription_content_filter_options_t * options) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_subscription_get_content_filter(
    const rmw_subscription_t * subscription,
    rcutils_allocator_t * allocator,
    rmw_subscription_content_filter_options_t * options) {
    return RMW_RET_UNSUPPORTED;
}
