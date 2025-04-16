#include "SharedUnitreeLidar.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <omp.h>
#include <cuda_runtime.h>
#include <iostream>

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

// CUDA kernel for point cloud transformation (placeholder)
__global__ void transformKernel(PointType* input, PointType* output, size_t numPoints) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < numPoints) {
        // Copy or transform the point (for now, it's a simple copy)
        output[idx] = input[idx];
    }
}

/**
 * @brief Transform a Unitree cloud to PCL cloud
 * 
 * @param cloudIn 
 * @param cloudOut 
 * @param useGPU Flag to decide whether to use CPU (OpenMP) or GPU (CUDA)
 */
 void transformUnitreeCloudToPCL(const unitree_lidar_sdk::PointCloudUnitree& cloudIn, pcl::PointCloud<PointType>::Ptr cloudOut, bool useGPU = false) {
    size_t numPoints = cloudIn.points.size();
    cloudOut->clear();
    cloudOut->reserve(numPoints);  // Pre-allocate memory to avoid reallocations

    // If GPU acceleration is enabled
    if (useGPU) {
        PointType *d_input, *d_output;

        // Allocate device memory
        cudaMalloc(&d_input, numPoints * sizeof(PointType));
        cudaMalloc(&d_output, numPoints * sizeof(PointType));

        // Copy input data to GPU
        cudaMemcpy(d_input, cloudIn.points.data(), numPoints * sizeof(PointType), cudaMemcpyHostToDevice);

        // Define block size and grid size
        int blockSize = 256;
        int numBlocks = (numPoints + blockSize - 1) / blockSize;

        // Launch CUDA kernel
        transformKernel<<<numBlocks, blockSize>>>(d_input, d_output, numPoints);

        // Copy output data back to host
        cudaMemcpy(cloudOut->points.data(), d_output, numPoints * sizeof(PointType), cudaMemcpyDeviceToHost);

        // Free GPU memory
        cudaFree(d_input);
        cudaFree(d_output);
    }
    else {
        // CPU Acceleration (OpenMP)
        PointType pt;

        // Temporary storage to avoid thread contention on push_back
        std::vector<PointType, Eigen::aligned_allocator<PointType>> localPoints(numPoints);

        // Parallelize the loop using OpenMP
        #pragma omp parallel for
        for (size_t i = 0; i < numPoints; i++) {
            pt.x = cloudIn.points[i].x;
            pt.y = cloudIn.points[i].y;
            pt.z = cloudIn.points[i].z;
            pt.intensity = cloudIn.points[i].intensity;
            pt.time = cloudIn.points[i].time;
            pt.ring = cloudIn.points[i].ring;

            localPoints[i] = pt;  // Store the result in local array
        }

        // Once all points are processed, add them to the cloudOut (avoiding thread contention)
        cloudOut->points = std::move(localPoints);  // Use move semantics for assignment
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
            // std::cout << framenumber << ";" << current - writetime << std::endl;
            auto start = std::chrono::high_resolution_clock::now();
            transformUnitreeCloudToPCL(lidar, cloud, false);
            // End timer
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> duration = end - start;
            std::cout << "Processing Time: " << duration.count() << " seconds" << std::endl;
        }
    }

    return 0;
}
