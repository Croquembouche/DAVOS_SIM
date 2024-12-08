#include "rmw/rmw.h"
#include "rmw/error_handling.h"
#include "SharedImage.h"
#include <opencv2/opencv.hpp>  // Include OpenCV headers


rmw_ret_t rmw_take(
  const rmw_subscription_t * subscription,
  cv::Mat & image,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  if (!subscription || !taken) {
    RMW_SET_ERROR_MSG("Invalid arguments in rmw_take");
    return RMW_RET_INVALID_ARGUMENT;
  }

  auto reader = static_cast<SharedImageReader *>(subscription->data);

  // Check if data is ready
  // if (!reader->isReadyToRead()) {
  //   
  //   return RMW_RET_OK;
  // }

  // Read from shared memory
  *taken = false;
  int last_frame = -1;
  if (!reader->readImageCPU(image, last_frame)) {
    RMW_SET_ERROR_MSG("Failed to read from shared memory");
    return RMW_RET_ERROR;
  }

  // Reset the "data-ready" flag
  // reader->resetReadyToRead();
  *taken = true;
  return RMW_RET_OK;
}
