#include "rmw_davos_dds/utilities.hpp"

const char * rmw_get_implementation_identifier() {
    return "rmw_davos_dds";
}

const char * rmw_get_serialization_format() {
    return "cdr";
}

rmw_ret_t rmw_set_log_severity(rmw_log_severity_t severity) {
    return RMW_RET_UNSUPPORTED;
}
