#include "rmw/init.h"
#include "rmw/error_handling.h"
#define RMW_IMPLEMENTATION_IDENTIFIER "rmw_davos_dds"

RMW_PUBLIC
// Define RMW_PUBLIC for symbol export
rmw_ret_t rmw_shutdown(rmw_context_t * context)
{
    if (!context) {
        RMW_SET_ERROR_MSG("context is null");
        return RMW_RET_INVALID_ARGUMENT;
    }

    // Perform middleware-specific shutdown tasks
    // (e.g., clean up domain participants, finalize network connections, etc.)

    context->impl = NULL; // Clean up context's implementation data
    return RMW_RET_OK;
}
