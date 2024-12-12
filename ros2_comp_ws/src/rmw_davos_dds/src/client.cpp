#include "rmw_davos_dds/client.hpp"

rmw_client_t * rmw_create_client(
    const rmw_node_t * node,
    const rosidl_service_type_support_t * type_support,
    const char * service_name,
    const rmw_qos_profile_t * qos_profile,
    const rmw_client_options_t * client_options) {
    return nullptr;
}

rmw_ret_t rmw_destroy_client(rmw_client_t * client) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_send_request(
    const rmw_client_t * client,
    const void * ros_request,
    int64_t * sequence_id) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_take_response(
    const rmw_client_t * client,
    rmw_service_info_t * request_header,
    void * ros_response,
    bool * taken) {
    return RMW_RET_UNSUPPORTED;
}
