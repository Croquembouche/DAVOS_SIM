#include "rmw/rmw.h"
#include "rmw/error_handling.h"
#include "SharedImage.h"

rmw_subscription_t *rmw_create_subscription(
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_support,
  const char * topic_name,
  const rmw_qos_profile_t * qos_policies)
{
  // Validate arguments
  if (!node || !type_support || !topic_name) {
    RMW_SET_ERROR_MSG("Invalid arguments in rmw_create_subscription");
    return nullptr;
  }

  // Allocate memory for the subscription
  auto * subscription = new rmw_subscription_t;
  subscription->topic_name = strdup(topic_name);

  if (!subscription->topic_name) {
    RMW_SET_ERROR_MSG("Failed to allocate memory for topic name");
    delete subscription;
    return nullptr;
  }

  try {
    // Create the shared memory reader for this subscription
    SharedImageReader * reader = new SharedImageReader(topic_name);

    // Associate the reader with the subscription
    subscription->data = reader;
    return subscription;

  } catch (const std::exception & e) {
    // Clean up in case of an error
    RMW_SET_ERROR_MSG(e.what());
    delete subscription;
    return nullptr;
  }
}
