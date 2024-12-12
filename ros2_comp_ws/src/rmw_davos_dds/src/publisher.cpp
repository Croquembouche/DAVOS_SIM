#include "rmw_davos_dds/publisher.hpp"

rmw_publisher_t * rmw_create_publisher(
    const rmw_node_t * node,
    const rosidl_message_type_support_t * type_support,
    const char * topic_name,
    const rmw_qos_profile_t * qos_profile,
    const rmw_publisher_options_t * publisher_options) {
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
    return publisher;
  } catch (const std::exception & e) {
      RMW_SET_ERROR_MSG(e.what());
      delete publisher;
      return nullptr;
  }
}

rmw_ret_t rmw_destroy_publisher(rmw_publisher_t * publisher) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_publish(
    const rmw_publisher_t * publisher,
    const void * ros_message,
    rmw_publisher_allocation_t * allocation) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_publish_serialized_message(
    const rmw_publisher_t * publisher,
    const rmw_serialized_message_t * serialized_message,
    rmw_publisher_allocation_t * allocation) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_serialize(
    const void * ros_message,
    const rosidl_message_type_support_t * type_support,
    rmw_serialized_message_t * serialized_message) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_publisher_get_actual_qos(
    const rmw_publisher_t * publisher,
    rmw_qos_profile_t * qos) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_publisher_assert_liveliness(const rmw_publisher_t * publisher) {
    return RMW_RET_UNSUPPORTED;
}
