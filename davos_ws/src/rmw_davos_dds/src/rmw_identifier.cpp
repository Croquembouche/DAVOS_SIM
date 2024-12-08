#include "rmw/rmw.h"

// This function returns the identifier of your custom RMW implementation.
const char * rmw_get_implementation_identifier()
{
  return "rmw_davos_dds";
}
