#ifndef RMW_DAVOS_DDS_PUBLISHER_HPP
#define RMW_DAVOS_DDS_PUBLISHER_HPP

#include <rmw/rmw.h>
#include "rmw/error_handling.h"
#include "SharedImage.h"


extern "C" {
    rmw_publisher_t * rmw_create_publisher(
        const rmw_node_t * node,
        const rosidl_message_type_support_t * type_support,
        const char * topic_name,
        const rmw_qos_profile_t * qos_profile,
        const rmw_publisher_options_t * publisher_options);
    rmw_ret_t rmw_destroy_publisher(rmw_publisher_t * publisher);
    rmw_ret_t rmw_publish(
        const rmw_publisher_t * publisher,
        const void * ros_message,
        rmw_publisher_allocation_t * allocation);
    rmw_ret_t rmw_publish_serialized_message(
        const rmw_publisher_t * publisher,
        const rmw_serialized_message_t * serialized_message,
        rmw_publisher_allocation_t * allocation);
    rmw_ret_t rmw_serialize(
        const void * ros_message,
        const rosidl_message_type_support_t * type_support,
        rmw_serialized_message_t * serialized_message);
    rmw_ret_t rmw_publisher_get_actual_qos(
        const rmw_publisher_t * publisher,
        rmw_qos_profile_t * qos);
    rmw_ret_t rmw_publisher_assert_liveliness(const rmw_publisher_t * publisher);
}

#endif // RMW_DAVOS_DDS_PUBLISHER_HPP
