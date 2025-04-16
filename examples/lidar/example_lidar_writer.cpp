#include <SharedLidar.h>
#include <cstring>
#include <chrono>
#include <unitree_lidar_sdk.h>

SharedLidarWriter* global_writer = nullptr;

int main(){

  // Initialize Lidar Object
  unitree_lidar_sdk::UnitreeLidarReader* lreader = unitree_lidar_sdk::createUnitreeLidarReader();
  int cloud_scan_num = 18;
  int point_byte = 24;
  int max_points = cloud_scan_num * 120;
  std::string port_name = "/dev/ttyUSB0";

  if ( lreader->initialize(cloud_scan_num, port_name) ){
    printf("Unilidar initialization failed! Exit here!\n");
    exit(-1);
  }else{
    printf("Unilidar initialization succeed!\n");
  }

  // Set Lidar Working Mode
  printf("Set Lidar working mode to: STANDBY ... \n");
  lreader->setLidarWorkingMode(unitree_lidar_sdk::STANDBY);
  sleep(1);

  printf("Set Lidar working mode to: NORMAL ... \n");
  lreader->setLidarWorkingMode(unitree_lidar_sdk::NORMAL);
  sleep(1);

  printf("\n");

  // Print Lidar Version
  while(true){
    if (lreader->runParse() == unitree_lidar_sdk::VERSION){
      printf("lidar firmware version = %s\n", lreader->getVersionOfFirmware().c_str() );
      break;
    }
    usleep(500);
  }
  printf("lidar sdk version = %s\n\n", lreader->getVersionOfSDK().c_str());
//   sleep(2);

  // Check lidar dirty percentange
  int count_percentage = 0;
  while(true){
    if( lreader->runParse() == unitree_lidar_sdk::AUXILIARY){
      printf("Dirty Percentage = %f %%\n", lreader->getDirtyPercentage());
      if (++count_percentage > 2){
        break;
      }
      if (lreader->getDirtyPercentage() > 10){
        printf("The protection cover is too dirty! Please clean it right now! Exit here ...\n");
        exit(0);
      }
    }
    usleep(100);
  }
  printf("\n");
//   sleep(2);

  // Parse PointCloud and IMU data
  unitree_lidar_sdk::MessageType result;
  std::string version;
  unitree_lidar_sdk::PointCloudUnitree cloudMsg;
  unitree_lidar_sdk::ScanUnitree scanMsg;
  char buffer[10000];
  uint32_t length = 0;
  bool imuMsgSent = false;
  bool scanMsgSent = false;
  uint32_t imuMsgType = 101;
  uint32_t scanMsgType = 102;

// Instantiate the shared memory writer
  char center_lidar[] = "center_lidar";
  global_writer = new SharedLidarWriter(center_lidar, max_points);

  printf("Data type size: \n");
  printf("\tsizeof(PointUnitree) = %ld\n", sizeof(unitree_lidar_sdk::PointUnitree));
  printf("\tsizeof(ScanUnitree) = %ld\n", sizeof(unitree_lidar_sdk::ScanUnitree));
  printf("\tsizeof(IMUUnitree) = %ld\n", sizeof(unitree_lidar_sdk::IMUUnitree));
  int iteration = 0;
  // auto start_ = std::chrono::high_resolution_clock::now();
  // int frameCount = 0;
  // int fps_ = 0;
  // auto end_ = std::chrono::high_resolution_clock::now();
  // std::chrono::duration<double> elapsed_ = end_ - start_;
  
  while (true)
  {
    result = lreader->runParse(); // You need to call this function at least 1500Hz

    switch (result)
    {
    case unitree_lidar_sdk::NONE:
      break;

    case unitree_lidar_sdk::IMU:
        // printf("Got imu");
    //   length = unitree_lidar_sdk::dataStructToUDPBuffer<unitree_lidar_sdk::IMUUnitree>(lreader->getIMU(), imuMsgType, buffer);
    //   client.Send(buffer, length, (char *)destination_ip.c_str(), destination_port); // 发送数据

    //   if (imuMsgSent == false)
    //   {
    //     imuMsgSent = true;
    //     printf("IMU message is sending!\n");
    //     printf("\tData format: | uint32_t msgType | uint32_t dataSize | IMUUnitree data |\n");
    //     printf("\tMsgType = %d, SentSize=%d, DataSize = %ld\n", imuMsgType, length, sizeof(IMUUnitree));
    //   }

        break;

    case unitree_lidar_sdk::POINTCLOUD:{
      cloudMsg = lreader->getCloud();
      // printf("\tLidarPointSize = %ld\n", cloudMsg.points.size());
      // frameCount++;
      // end_ = std::chrono::high_resolution_clock::now();
      // elapsed_ = end_ - start_;
      // if (elapsed_.count() >= 1.0) {
      //     fps_ = frameCount / elapsed_.count();
      //     std::cout << "FPS:" << fps_ << std::endl;
      //     frameCount = 0;
      //     start_ = std::chrono::high_resolution_clock::now();
      // }
      // printf("got message");
      // scanMsg.id = cloudMsg.id;
      // scanMsg.stamp = cloudMsg.stamp;
      // scanMsg.validPointsNum = cloudMsg.points.size();
      // memcpy(scanMsg.points, cloudMsg.points.data(), cloudMsg.points.size() * sizeof(PointUnitree));
      // Write the image to shared memory
      // Convert to PCL cloud
      pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
      pcl_cloud.points.reserve(cloudMsg.points.size());
      
      for (const auto& pt : cloudMsg.points) {
          pcl_cloud.points.emplace_back(pcl::PointXYZ{
              pt.x, pt.y, pt.z
          });
      }
      
      pcl_cloud.width = pcl_cloud.size();
      pcl_cloud.height = 1;
      pcl_cloud.is_dense = true;
      if (global_writer->writePointCloud(pcl_cloud)) {
          std::cout << "Writer: Lidar written to shared memory (version " << iteration << ")" << std::endl;
          iteration += 1;
      } else {
          std::cerr << "Writer: Failed to write lidar to shared memory" << std::endl;
      }
      break;
    }

    default:
      // printf("Nothing is happending\n");
      break;
    }

    usleep(1000);
  }
  delete global_writer;
  return 0;
}
