#include "rmw_davos_dds/service.hpp"

rmw_service_t * rmw_create_service(
    const rmw_node_t * node,
    const rosidl_service_type_support_t * type_support,
    const char * service_name,
    const rmw_qos_profile_t * qos_profile,
    const rmw_service_options_t * service_options) {
    return nullptr;
}

rmw_ret_t rmw_destroy_service(rmw_service_t * service) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_take_request(
    const rmw_service_t * service,
    rmw_service_info_t * request_header,
    void * ros_request,
    bool * taken) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_send_response(
    const rmw_service_t * service,
    rmw_request_id_t * request_header,
    void * ros_response) {
    return RMW_RET_UNSUPPORTED;
}
