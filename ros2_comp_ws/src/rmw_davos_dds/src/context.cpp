#include "rmw_davos_dds/context.hpp"

rmw_ret_t rmw_init_options_init(rmw_init_options_t * init_options, rcutils_allocator_t allocator) {
        // Validate input arguments
    if (!init_options) {
        RMW_SET_ERROR_MSG("init_options is null");
        return RMW_RET_INVALID_ARGUMENT;
    }

    if (!allocator.allocate || !allocator.deallocate) {
        RMW_SET_ERROR_MSG("allocator is invalid");
        return RMW_RET_INVALID_ARGUMENT;
    }

    // Initialize the rmw_init_options_t structure
    init_options->implementation_identifier = "rmw_davos_dds"; // Custom RMW identifier
    init_options->instance_id = 0; // Set a default or specific instance ID
    init_options->allocator = allocator;

    // Optional: Initialize domain-specific options
    // init_options->domain_id = RMW_DEFAULT_DOMAIN_ID;
    // init_options->security_options = (rmw_security_options_t){
    //     .enforce_security = RMW_SECURITY_ENFORCEMENT_PERMISSIVE,
    //     .security_root_path = NULL
    // };

    // Additional custom initialization logic, if required
    // For example, setting specific configuration for Davos DDS

    return RMW_RET_OK;
}

rmw_ret_t rmw_init_options_copy(const rmw_init_options_t * src, rmw_init_options_t * dst) {
    if (!src || !dst) {
    RMW_SET_ERROR_MSG("Invalid argument");
    return RMW_RET_INVALID_ARGUMENT;
  }

  *dst = *src; // Copy structure contents
  dst->implementation_identifier = "rmw_davos_dds"; // Ensure correct identifier

  // Allocate memory for security root path, if needed
  if (src->security_options.security_root_path) {
    dst->security_options.security_root_path = strdup(src->security_options.security_root_path);
    if (!dst->security_options.security_root_path) {
      RMW_SET_ERROR_MSG("Failed to allocate memory for security root path");
      return RMW_RET_BAD_ALLOC;
    }
  }

  return RMW_RET_OK;
}

rmw_ret_t rmw_init_options_fini(rmw_init_options_t * init_options) {
    // Validate input arguments
    if (!init_options) {
        RMW_SET_ERROR_MSG("init_options is null");
        return RMW_RET_INVALID_ARGUMENT;
    }

    // if (!allocator.allocate || !allocator.deallocate) {
    //     RMW_SET_ERROR_MSG("allocator is invalid");
    //     return RMW_RET_INVALID_ARGUMENT;
    // }

    // Initialize the rmw_init_options_t structure
    init_options->implementation_identifier = "rmw_davos_dds"; // Custom RMW identifier
    init_options->instance_id = 0; // Set a default or specific instance ID
    // init_options->allocator = allocator;

    // Optional: Initialize domain-specific options
    // init_options->domain_id = RMW_DEFAULT_DOMAIN_ID;
    // init_options->security_options = (rmw_security_options_t){
    //     .enforce_security = RMW_SECURITY_ENFORCEMENT_PERMISSIVE,
    //     .security_root_path = NULL
    // };

    // Additional custom initialization logic, if required
    // For example, setting specific configuration for Davos DDS

    return RMW_RET_OK;
}

rmw_ret_t rmw_init(const rmw_init_options_t * options, rmw_context_t * context) {
    return RMW_RET_OK;
}

rmw_ret_t rmw_shutdown(rmw_context_t * context) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_context_fini(rmw_context_t * context) {
    if (!context) {
        RMW_SET_ERROR_MSG("context is null");
        return RMW_RET_INVALID_ARGUMENT;
    }

    if (context->impl) {
        RMW_SET_ERROR_MSG("context implementation is not null");
        return RMW_RET_ERROR;
    }

    return RMW_RET_OK;
}
