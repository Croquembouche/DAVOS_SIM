#include "rmw/rmw.h"
#include "rmw/error_handling.h"
#include "SharedImage.h"

rmw_publisher_t *rmw_create_publisher(
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_support,
  const char * topic_name,
  const rmw_qos_profile_t * qos_policies)
{
  if (!node || !type_support || !topic_name) {
    RMW_SET_ERROR_MSG("Invalid arguments in rmw_create_publisher");
    return nullptr;
  }

  auto * publisher = new rmw_publisher_t;
  publisher->topic_name = strdup(topic_name);

  // Initialize shared memory writer
  try {
    SharedImageWriter * writer = new SharedImageWriter(topic_name);
    publisher->data = writer;  // Associate the writer with the publisher
    publisher->data = writer;
    return publisher;
  } catch (const std::exception & e) {
      RMW_SET_ERROR_MSG(e.what());
      delete publisher;
      return nullptr;
  }



}
