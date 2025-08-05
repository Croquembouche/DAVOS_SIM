#include "SharedLidar.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <iostream>
#include <unitree_lidar_sdk.h>
#include <chrono>
#include <thread>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

#define SERVER_IP "128.175.213.244"  // IP of Raspberry Pi
#define SERVER_PORT 5005          // Port to connect to

bool sendPointCloud(int sock, const pcl::PointCloud<pcl::PointXYZ>& cloud, uint64_t timestamp_ns) {
    uint32_t num_points = cloud.size();

    // Send timestamp first
    if (send(sock, &timestamp_ns, sizeof(timestamp_ns), 0) != sizeof(timestamp_ns)) {
        std::cerr << "Failed to send timestamp\n";
        return false;
    }

    // Send number of points
    if (send(sock, &num_points, sizeof(num_points), 0) != sizeof(num_points)) {
        std::cerr << "Failed to send point count\n";
        return false;
    }

    // Send point data
    if (num_points > 0) {
        size_t byte_count = num_points * sizeof(pcl::PointXYZ);
        if (send(sock, reinterpret_cast<const char*>(cloud.points.data()), byte_count, 0)
            != static_cast<ssize_t>(byte_count)) {
            std::cerr << "Failed to send point data\n";
            return false;
        }
    }

    return true;
}


int main() {
    // ----------- TCP INIT -------------
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        std::cerr << "Socket creation failed\n";
        return 1;
    }

    sockaddr_in serverAddr{};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(SERVER_PORT);
    inet_pton(AF_INET, SERVER_IP, &serverAddr.sin_addr);

    if (connect(sock, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        std::cerr << "Connection to Raspberry Pi failed\n";
        return 1;
    }

    std::cout << "Connected to Raspberry Pi at " << SERVER_IP << "\n";

    // ----------- LIDAR INIT -------------
    int channels = 18;
    const int max_points = channels * 120;
    SharedLidarReader reader("center_lidar", max_points);

    if (!reader.isInitialized()) {
        std::cerr << "Lidar Reader init failed!" << std::endl;
        return 1;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    int last_frame = -1;

    // ----------- MAIN LOOP -------------
    while (true) {
        if (reader.readPointCloud(cloud, last_frame)) {
            auto framenumber = reader.getFrameNumber();
            uint64_t ts = reader.getTimeStamp();
            std::cout << "read point cloud width" << std::endl;
            std::cout << cloud.width << " " << cloud.height << std::endl;

            if (!sendPointCloud(sock, cloud, ts)) {
                std::cerr << "Send failed. Exiting.\n";
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    close(sock);
    return 0;
}
