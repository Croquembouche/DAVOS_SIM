#ifndef RMW_DAVOS_DDS_WAITSET_HPP
#define RMW_DAVOS_DDS_WAITSET_HPP

#include <rmw/rmw.h>
#include "rmw/error_handling.h"


extern "C" {
    rmw_wait_set_t * rmw_create_wait_set(const rmw_context_t * context, size_t max_conditions);
    rmw_ret_t rmw_destroy_wait_set(rmw_wait_set_t * wait_set);
    rmw_ret_t rmw_wait(
        rmw_subscriptions_t * subscriptions,
        rmw_guard_conditions_t * guard_conditions,
        rmw_services_t * services,
        rmw_clients_t * clients,
        rmw_events_t * events,
        rmw_wait_set_t * wait_set,
        const rmw_time_t * wait_timeout);
}

#endif // RMW_DAVOS_DDS_WAITSET_HPP
