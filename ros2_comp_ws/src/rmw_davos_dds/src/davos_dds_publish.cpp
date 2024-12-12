#include "rmw/rmw.h"
#include "rmw/error_handling.h"
#include "SharedImage.h"
#include <opencv2/opencv.hpp>  // Include OpenCV headers
#define RMW_IMPLEMENTATION_IDENTIFIER "rmw_davos_dds"

rmw_ret_t rmw_publish(
  const rmw_publisher_t * publisher,
  cv::Mat &image,
  rmw_publisher_allocation_t * allocation)
{
  if (!publisher) {
    RMW_SET_ERROR_MSG("Invalid arguments in rmw_publish");
    return RMW_RET_INVALID_ARGUMENT;
  }

  auto writer = static_cast<SharedImageWriter *>(publisher->data);

  // Write data to shared memory
  if (!writer->writeImageCPU(image)) {
    RMW_SET_ERROR_MSG("Failed to write to shared memory");
    return RMW_RET_ERROR;
  }

  // Set the "data-ready" flag
  // writer->setReadyToRead();
  return RMW_RET_OK;
}
