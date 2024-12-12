#ifndef RMW_DAVOS_DDS_CONTEXT_HPP
#define RMW_DAVOS_DDS_CONTEXT_HPP

#include <rmw/rmw.h>
#include "rmw/error_handling.h"
#include "rmw/init_options.h"
#include "rmw/error_handling.h"
#include "rmw/types.h"
#include "rcutils/allocator.h"
#include "rcutils/error_handling.h"

extern "C" {
    rmw_ret_t rmw_init(const rmw_init_options_t * options, rmw_context_t * context);
    rmw_ret_t rmw_shutdown(rmw_context_t * context);
    rmw_ret_t rmw_context_fini(rmw_context_t * context);
    rmw_ret_t rmw_init_options_init(rmw_init_options_t * init_options, rcutils_allocator_t allocator);
    rmw_ret_t rmw_init_options_copy(const rmw_init_options_t * src, rmw_init_options_t * dst);
    rmw_ret_t rmw_init_options_fini(rmw_init_options_t * init_options);
}

#endif // RMW_DAVOS_DDS_CONTEXT_HPP
