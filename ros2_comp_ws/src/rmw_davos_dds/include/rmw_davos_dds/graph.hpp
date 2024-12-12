#ifndef RMW_CUSTOM_DDS_GRAPH_HPP
#define RMW_CUSTOM_DDS_GRAPH_HPP

#include <rmw/rmw.h>

extern "C" {
    rmw_ret_t rmw_get_node_names(
        const rmw_node_t * node,
        rcutils_string_array_t * node_names,
        rcutils_string_array_t * node_namespaces);
    rmw_ret_t rmw_get_node_names_with_enclaves(
        const rmw_node_t * node,
        rcutils_string_array_t * node_names,
        rcutils_string_array_t * node_namespaces,
        rcutils_string_array_t * enclaves);
    rmw_ret_t rmw_get_topic_names_and_types(
        const rmw_node_t * node,
        rcutils_allocator_t * allocator,
        bool no_demangle,
        rmw_names_and_types_t * topic_names_and_types);
    rmw_ret_t rmw_get_service_names_and_types(
        const rmw_node_t * node,
        rcutils_allocator_t * allocator,
        rmw_names_and_types_t * service_names_and_types);
}

#endif // RMW_CUSTOM_DDS_GRAPH_HPP
