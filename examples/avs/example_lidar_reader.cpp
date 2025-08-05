#include "SharedLidar.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <unitree_lidar_sdk.h>
#include <chrono>
#include <algorithm>
#include <thread>
#include <vector>
#include <string>

cv::Mat BirdsEyeView(const pcl::PointCloud<pcl::PointXYZ>& input_cloud,
  const std::pair<float, float>& xy_range = {-5, 5},
  const std::pair<float, float>& z_range = {0, 1.2},
  float res = 0.1) {
  
  const float xy_min = xy_range.first;
  const float xy_max = xy_range.second;
  const float z_min = z_range.first;
  const float z_max = z_range.second;

  int img_width = static_cast<int>((xy_max - xy_min) / res);

  cv::Mat img = cv::Mat::zeros(img_width, img_width, CV_8UC1);

  for (const auto& pt : input_cloud.points) {
      // Apply combined filtering in one pass
      if (pt.x < xy_min || pt.x > xy_max || pt.y < xy_min || pt.y > xy_max || pt.z < z_min || pt.z > z_max)
          continue;

      int x_pixel = static_cast<int>((pt.x - xy_min) / res);
      int y_pixel = static_cast<int>((pt.y - xy_min) / res);

      // Flip X for top-down view
      x_pixel = img_width - x_pixel - 1;

      if (x_pixel >= 0 && x_pixel < img_width && y_pixel >= 0 && y_pixel < img_width) {
          uint8_t pixel_value = static_cast<uint8_t>(255.0f * (pt.z - z_min) / (z_max - z_min));
          img.at<uint8_t>(y_pixel, x_pixel) = std::max(img.at<uint8_t>(y_pixel, x_pixel), pixel_value);
      }
  }

  return img;
}


// ==============================================================================
// Histogram Functions
// ==============================================================================
double CompareHist(const cv::Mat& img1, const cv::Mat& img2) {
  cv::Mat hist1, hist2;
  int histSize = 256;
  float range[] = { 0, 256 };
  const float* histRange = { range };

  cv::calcHist(&img1, 1, 0, cv::Mat(), hist1, 1, &histSize, &histRange);
  cv::normalize(hist1, hist1, 0, 1, cv::NORM_MINMAX);
  cv::calcHist(&img2, 1, 0, cv::Mat(), hist2, 1, &histSize, &histRange);
  cv::normalize(hist2, hist2, 0, 1, cv::NORM_MINMAX);

  return cv::compareHist(hist1, hist2, cv::HISTCMP_CHISQR);
}



int main() {
  const int channels = 18;
  const int max_points = channels * 120;
  SharedLidarReader reader("center_lidar", max_points);

  if (!reader.isInitialized()) {
      std::cerr << "Reader init failed!" << std::endl;
      return 1;
  }

  pcl::PointCloud<pcl::PointXYZ> cloud, temp_cloud;
  int last_frame = -1;
  int frame_counter = 0;
  cv::Mat stored_bev;

  // cv::Mat hist1, hist2;
  // int histSize = 256;
  // float range[] = {0, 256};
  // const float* histRange = {range};

  // const float chi_threshold = 0.01f;

  while (true) {
      // Read frame
      if (!reader.readPointCloud(cloud, last_frame))
          continue;

      std::cout << cloud.width << " " << cloud.is_dense << " " << cloud.height << std::endl;

    // Point Cloud buffer to 360 degrees frame
    //   temp_cloud += cloud;
    //   frame_counter++;

    //   if (frame_counter < 5)
    //       continue;

    //   // Once we have 5 frames, generate BEV
    //   cv::Mat bev = BirdsEyeView(temp_cloud);
    //   temp_cloud.clear();
    //   frame_counter = 0;

    //   auto framenumber = reader.getFrameNumber();
    //   std::ostringstream filename;
    //   filename << "bev_frame_" << framenumber << ".png";
    //   // Store bev
    //   cv::imwrite(filename.str(), bev);
    //   std::cout << "Save BEV done!"<< std::endl;

      // if (stored_bev.empty()) {
      //     stored_bev = bev.clone();
      //     std::cout << "saved first frame\n";
      //     continue;
      // }

      // // Compare histograms
      // cv::Mat hist1, hist2;
      // int histSize = 256;
      // float range[] = {0, 256};
      // const float* histRange = {range};

      // cv::calcHist(&stored_bev, 1, 0, cv::Mat(), hist1, 1, &histSize, &histRange, true, false);
      // cv::calcHist(&bev, 1, 0, cv::Mat(), hist2, 1, &histSize, &histRange, true, false);

      // double chisqr = cv::compareHist(hist1, hist2, cv::HISTCMP_CHISQR);
      // std::cout << chisqr << "\n";
      // stored_bev = bev.clone();

      // if (chisqr > chi_threshold) {
      //     stored_bev = bev.clone();
      //     output_file << last_frame << "\n";
      //     std::cout << "Recorded frame: " << last_frame << " (ChiSq=" << chisqr << ")\n";
      // }
  }

  return 0;
}
