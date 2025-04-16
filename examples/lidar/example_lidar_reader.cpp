#include "SharedLidar.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <iostream>
#include <unitree_lidar_sdk.h>
#include <chrono>
#include <thread>




int main() {

  int channels = 18;
  const int max_points = channels * 120;  // Must match writer
  SharedLidarReader reader("center_lidar", max_points);
  
  if (!reader.isInitialized()) {
      std::cerr << "Reader init failed!" << std::endl;
      return 1;
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  int last_frame = -1;
  int last_framenumber = 0;
  
  while (true) {
      if (reader.readPointCloud(cloud, last_frame)) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        auto current = ts.tv_sec * 1e9 + ts.tv_nsec;
        auto writetime = reader.getTimeStamp();
        auto framenumber = reader.getFrameNumber();
        // std::cout << "Reader: New image read from shared memory (version " << last_frame << ")" << std::endl;
        if (framenumber-last_framenumber != 1){
            std::cout << "Missed: " << framenumber << ";" << last_framenumber << std::endl;
        }
        last_framenumber = framenumber;
        std::cout << framenumber << ":" << (current - writetime)/1000000 << std::endl;
          // std::cout << "New frame " << last_frame 
          //           << " with " << cloud.size() 
          //           << " points (ts: " << reader.getTimeStamp()
          //           << ")" << std::endl;
      }
      std::this_thread::sleep_for(std::chrono::microseconds(100)); // Requests 0.1 ms sleep
  }
  
  return 0;
}
