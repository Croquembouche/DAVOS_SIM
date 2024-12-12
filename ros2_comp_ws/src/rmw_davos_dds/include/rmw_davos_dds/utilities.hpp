#ifndef RMW_DAVOS_DDS_UTILITIES_HPP
#define RMW_DAVOS_DDS_UTILITIES_HPP

#include <rmw/rmw.h>
#include "rmw/error_handling.h"


extern "C" {
    const char * rmw_get_implementation_identifier();
    const char * rmw_get_serialization_format();
    rmw_ret_t rmw_set_log_severity(rmw_log_severity_t severity);
}

#endif // RMW_DAVOS_DDS_UTILITIES_HPP
