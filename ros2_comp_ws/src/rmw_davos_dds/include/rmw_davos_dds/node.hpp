#ifndef RMW_DAVOS_DDS_NODE_HPP
#define RMW_DAVOS_DDS_NODE_HPP

#include <rmw/rmw.h>
#include "rmw/error_handling.h"


extern "C" {
    rmw_node_t * rmw_create_node(
        const rmw_context_t * context,
        const char * name,
        const char * namespace_,
        size_t domain_id,
        bool localhost_only);
    rmw_ret_t rmw_destroy_node(rmw_node_t * node);
    const rmw_guard_condition_t * rmw_node_get_graph_guard_condition(const rmw_node_t * node);

}

#endif // RMW_DAVOS_DDS_NODE_HPP
