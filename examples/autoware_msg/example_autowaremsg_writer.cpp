#include "SharedAutowareMSG.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <signal.h>
#include <rclcpp/rclcpp.hpp>

// Flag to control the main loop
bool running = true;

// Signal handler for graceful shutdown
void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received. Exiting gracefully..." << std::endl;
    running = false;
}

int main(int argc, char* argv[]) {
    // Register signal handler
    signal(SIGINT, signalHandler);

    // Initialize ROS
    rclcpp::init(argc, argv);

    try {
        // Create shared memory writer
        const std::string shm_name = "autoware_msgs";
        const size_t shm_size = 100 * 1024 * 1024; // 100 MB
        davos::shm::AutowareSharedMemoryWriter writer(shm_name, shm_size);

        std::cout << "SharedAutowareMSG Writer started. Press Ctrl+C to exit." << std::endl;
        
        // Example messages
        autoware_auto_vehicle_msgs::msg::VehicleKinematicState kinematic_state_msg;
        autoware_auto_vehicle_msgs::msg::VehicleControlCommand control_command_msg;
        autoware_auto_planning_msgs::msg::Trajectory trajectory_msg;
        
        // Counter for simulating changing data
        uint32_t counter = 0;
        
        // Main loop
        while (running) {
            counter++;
            
            // Fill kinematic state message
            kinematic_state_msg.header.stamp = rclcpp::Clock().now();
            kinematic_state_msg.header.frame_id = "base_link";
            kinematic_state_msg.state.x = static_cast<float>(counter % 100);
            kinematic_state_msg.state.y = static_cast<float>((counter + 50) % 100);
            kinematic_state_msg.state.heading.real = 1.0f;
            kinematic_state_msg.state.heading.imag = 0.0f;
            kinematic_state_msg.state.longitudinal_velocity_mps = 5.0f;
            
            // Fill control command message
            control_command_msg.stamp = rclcpp::Clock().now();
            control_command_msg.long_accel_mps2 = 0.5f;
            control_command_msg.front_wheel_angle_rad = 0.1f;
            control_command_msg.rear_wheel_angle_rad = 0.0f;
            control_command_msg.velocity_mps = 10.0f + (counter % 5);
            
            // Fill trajectory message
            trajectory_msg.header.stamp = rclcpp::Clock().now();
            trajectory_msg.header.frame_id = "map";
            
            // Clear and add points to trajectory
            trajectory_msg.points.clear();
            for (int i = 0; i < 10; i++) {
                autoware_auto_planning_msgs::msg::TrajectoryPoint point;
                point.x = static_cast<float>(i * 2 + (counter % 5));
                point.y = static_cast<float>(i + (counter % 3));
                point.longitudinal_velocity_mps = 10.0f - (i * 0.5f);
                trajectory_msg.points.push_back(point);
            }
            
            // Write messages to shared memory
            bool success_kin = writer.write(davos::shm::AutowareMsgType::VehicleKinematicState, kinematic_state_msg);
            bool success_cmd = writer.write(davos::shm::AutowareMsgType::VehicleControlCommand, control_command_msg);
            bool success_traj = writer.write(davos::shm::AutowareMsgType::Trajectory, trajectory_msg);
            
            std::cout << "Write cycle " << counter << ": "
                      << "Kinematic State: " << (success_kin ? "OK" : "FAIL") << ", "
                      << "Control Command: " << (success_cmd ? "OK" : "FAIL") << ", "
                      << "Trajectory: " << (success_traj ? "OK" : "FAIL") << std::endl;
            
            // Sleep for a while
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        std::cout << "Writer shutting down..." << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
