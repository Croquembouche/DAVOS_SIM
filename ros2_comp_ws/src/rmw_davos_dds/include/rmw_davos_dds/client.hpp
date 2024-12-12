#ifndef RMW_DAVOS_DDS_CLIENT_HPP
#define RMW_DAVOS_DDS_CLIENT_HPP

#include <rmw/rmw.h>
#include "rmw/error_handling.h"

extern "C" {
    rmw_client_t * rmw_create_client(
        const rmw_node_t * node,
        const rosidl_service_type_support_t * type_support,
        const char * service_name,
        const rmw_qos_profile_t * qos_profile,
        const rmw_client_options_t * client_options);
    rmw_ret_t rmw_destroy_client(rmw_client_t * client);
    rmw_ret_t rmw_send_request(
        const rmw_client_t * client,
        const void * ros_request,
        int64_t * sequence_id);
    rmw_ret_t rmw_take_response(
        const rmw_client_t * client,
        rmw_service_info_t * request_header,
        void * ros_response,
        bool * taken);
}

#endif // RMW_DAVOS_DDS_CLIENT_HPP
