#include "rmw_custom_dds/graph.hpp"

rmw_ret_t rmw_get_node_names(
    const rmw_node_t * node,
    rcutils_string_array_t * node_names,
    rcutils_string_array_t * node_namespaces) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_get_node_names_with_enclaves(
    const rmw_node_t * node,
    rcutils_string_array_t * node_names,
    rcutils_string_array_t * node_namespaces,
    rcutils_string_array_t * enclaves) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_get_topic_names_and_types(
    const rmw_node_t * node,
    rcutils_allocator_t * allocator,
    bool no_demangle,
    rmw_names_and_types_t * topic_names_and_types) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_get_service_names_and_types(
    const rmw_node_t * node,
    rcutils_allocator_t * allocator,
    rmw_names_and_types_t * service_names_and_types) {
    return RMW_RET_UNSUPPORTED;
}
