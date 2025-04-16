#include "SharedUnitreeLidar.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

int point_byte = 24;
int cloud_scan_num = 18;


/**
 * @brief PCL Point Type
 */
struct PointType 
{
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY
  std::uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}  ;  // Explicitly align to 16 bytes
POINT_CLOUD_REGISTER_POINT_STRUCT(PointType,
  (float, x, x)(float, y, y)(float, z, z)
  (float, intensity, intensity)
  (std::uint16_t, ring, ring)
  (float, time, time)
)

/**
 * @brief Transform a Unitree cloud to PCL cloud
 * 
 * @param cloudIn 
 * @param cloudOut 
 */
void transformUnitreeCloudToPCL(const unitree_lidar_sdk::PointCloudUnitree& cloudIn, pcl::PointCloud<PointType>::Ptr cloudOut){
  cloudOut->clear();
  PointType pt;
  for (size_t i = 0; i < cloudIn.points.size(); i++) {
    pt.x = cloudIn.points[i].x;
    pt.y = cloudIn.points[i].y;
    pt.z = cloudIn.points[i].z;
    pt.intensity = cloudIn.points[i].intensity;
    pt.time = cloudIn.points[i].time;
    pt.ring = cloudIn.points[i].ring;
    cloudOut->push_back(pt);
  }
}

int main() {
    // Instantiate the shared memory reader
    char center_lidar[] = "center_lidar";
    SharedUnitreeLidarReader reader(center_lidar, cloud_scan_num, point_byte);
    unitree_lidar_sdk::PointCloudUnitree lidar;

    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);

    // Check if shared memory mapping was successful
    if (!reader.isInitialized()) {
        std::cerr << "Failed to initialize shared memory reader. Exiting." << std::endl;
        return 1;  // Exit if shared memory setup failed
    }

    int last_frame = -1;

    auto start = std::chrono::high_resolution_clock::now();
    int frameCount = 0;
    int last_framenumber = 0;

    while (true) {
        // Attempt to read a new image from shared memory
        if (reader.readLidar(lidar, last_frame)) {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            auto current = ts.tv_sec * 1e9 + ts.tv_nsec;
            auto writetime = reader.getTimeStamp();
            auto framenumber = reader.getFrameNumber();
            if (framenumber-last_framenumber != 1){
                std::cout << "Missed: " << framenumber << ";" << last_framenumber  << std::endl;
            }
            last_framenumber = framenumber;
            std::cout << framenumber << ";" << current - writetime << std::endl;
            //transformUnitreeCloudToPCL(lidar, cloud);

            // Specify the file path to save the PCD file
             //std::string file_path = "output_point_cloud.pcd";

            // Save the point cloud to disk in PCD format
            //if (pcl::io::savePCDFileASCII(file_path, *cloud) == -1) {
             //   PCL_ERROR("Couldn't save the PCD file\n");
             //   return (-1);
            //}
            // printf("\tstamp = %f", lidar.stamp);

             //std::cout << "PointCloud data saved to: " << file_path << std::endl;
             //break;
        }
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return 0;
}
