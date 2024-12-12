#include "rmw_davos_dds/node.hpp"

rmw_node_t * rmw_create_node(
    const rmw_context_t * context,
    const char * name,
    const char * namespace_,
    size_t domain_id,
    bool localhost_only) 
{
    if (!context || !name || !namespace_) {
        RMW_SET_ERROR_MSG("Invalid argument");
        return NULL;
    }

    // Allocate and initialize a new node
    rmw_node_t * node = static_cast<rmw_node_t *>(malloc(sizeof(rmw_node_t)));
    if (!node) {
        RMW_SET_ERROR_MSG("Failed to allocate memory for node");
        return NULL;
    }

    node->implementation_identifier = "rmw_davos_dds"; // Set your RMW identifier
    node->name = strdup(name);
    node->namespace_ = strdup(namespace_);
    node->context = context;

    // Perform DDS-specific node creation (e.g., create a participant)
    // Add custom participant initialization here

    return node;
}

rmw_ret_t rmw_destroy_node(rmw_node_t * node) {
    if (!node) {
        RMW_SET_ERROR_MSG("node is null");
        return RMW_RET_INVALID_ARGUMENT;
    }

    // Perform DDS-specific cleanup for the node (e.g., destroy participant)
    // Add custom cleanup logic here

    free(const_cast<char *>(node->name));
    free(const_cast<char *>(node->namespace_));
    free(node);
    return RMW_RET_OK;
}

const rmw_guard_condition_t * rmw_node_get_graph_guard_condition(const rmw_node_t * node) {
    return nullptr;
}
