#ifndef RMW_CUSTOM_DDS_GUARD_CONDITION_HPP
#define RMW_CUSTOM_DDS_GUARD_CONDITION_HPP

#include <rmw/rmw.h>

extern "C" {
    rmw_guard_condition_t * rmw_create_guard_condition(rmw_context_t * context);
    rmw_ret_t rmw_destroy_guard_condition(rmw_guard_condition_t * guard_condition);
    rmw_ret_t rmw_trigger_guard_condition(const rmw_guard_condition_t * guard_condition);
}

#endif // RMW_CUSTOM_DDS_GUARD_CONDITION_HPP
