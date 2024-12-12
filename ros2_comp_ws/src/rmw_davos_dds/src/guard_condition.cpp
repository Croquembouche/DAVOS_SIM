#include "rmw_custom_dds/guard_condition.hpp"

rmw_guard_condition_t * rmw_create_guard_condition(rmw_context_t * context) {
    return nullptr;
}

rmw_ret_t rmw_destroy_guard_condition(rmw_guard_condition_t * guard_condition) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_trigger_guard_condition(const rmw_guard_condition_t * guard_condition) {
    return RMW_RET_UNSUPPORTED;
}
