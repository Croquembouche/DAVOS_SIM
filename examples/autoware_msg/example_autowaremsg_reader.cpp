#include "SharedAutowareMSG.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <signal.h>
#include <iomanip>

// Flag to control the main loop
bool running = true;

// Signal handler for graceful shutdown
void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received. Exiting gracefully..." << std::endl;
    running = false;
}

// Helper function to print kinematic state details
void printKinematicState(const autoware_auto_vehicle_msgs::msg::VehicleKinematicState& msg) {
    std::cout << "  Position: (" << msg.state.x << ", " << msg.state.y << ")" << std::endl;
    std::cout << "  Velocity: " << msg.state.longitudinal_velocity_mps << " m/s" << std::endl;
    std::cout << "  Heading: (" << msg.state.heading.real << ", " << msg.state.heading.imag << ")" << std::endl;
}

// Helper function to print control command details
void printControlCommand(const autoware_auto_vehicle_msgs::msg::VehicleControlCommand& msg) {
    std::cout << "  Acceleration: " << msg.long_accel_mps2 << " m/s²" << std::endl;
    std::cout << "  Velocity: " << msg.velocity_mps << " m/s" << std::endl;
    std::cout << "  Wheel angle: " << msg.front_wheel_angle_rad << " rad" << std::endl;
}

// Helper function to print trajectory details
void printTrajectory(const autoware_auto_planning_msgs::msg::Trajectory& msg) {
    std::cout << "  Points: " << msg.points.size() << std::endl;
    if (!msg.points.empty()) {
        auto& p = msg.points[0];
        std::cout << "  First point: (" << p.x << ", " << p.y 
                  << "), v=" << p.longitudinal_velocity_mps << " m/s" << std::endl;
    }
}

int main() {
    // Register signal handler
    signal(SIGINT, signalHandler);

    try {
        // Create shared memory reader
        const std::string shm_name = "autoware_msgs";
        const size_t shm_size = 100 * 1024 * 1024; // 100 MB
        davos::shm::AutowareSharedMemoryReader reader(shm_name, shm_size);

        std::cout << "SharedAutowareMSG Reader started. Press Ctrl+C to exit." << std::endl;
        
        // Message objects
        autoware_auto_vehicle_msgs::msg::VehicleKinematicState kinematic_state_msg;
        autoware_auto_vehicle_msgs::msg::VehicleControlCommand control_command_msg;
        autoware_auto_planning_msgs::msg::Trajectory trajectory_msg;
        
        // For tracking sequence numbers
        uint64_t kin_seq = 0, cmd_seq = 0, traj_seq = 0;
        
        // Main loop
        while (running) {
            std::cout << "\n--- Reading Shared Memory ---" << std::endl;
            
            // Read kinematic state
            uint64_t new_kin_seq;
            bool success_kin = reader.read(
                davos::shm::AutowareMsgType::VehicleKinematicState, 
                kinematic_state_msg, 
                &new_kin_seq
            );
            
            if (success_kin) {
                if (new_kin_seq != kin_seq) {
                    kin_seq = new_kin_seq;
                    std::cout << "VehicleKinematicState (UPDATED):" << std::endl;
                    std::cout << "  Sequence: " << kin_seq << std::endl;
                    std::cout << "  Frame ID: " << kinematic_state_msg.header.frame_id << std::endl;
                    printKinematicState(kinematic_state_msg);
                } else {
                    std::cout << "VehicleKinematicState (no update)" << std::endl;
                }
            } else {
                std::cout << "VehicleKinematicState: READ FAILED" << std::endl;
            }
            
            // Read control command
            uint64_t new_cmd_seq;
            bool success_cmd = reader.read(
                davos::shm::AutowareMsgType::VehicleControlCommand, 
                control_command_msg, 
                &new_cmd_seq
            );
            
            if (success_cmd) {
                if (new_cmd_seq != cmd_seq) {
                    cmd_seq = new_cmd_seq;
                    std::cout << "VehicleControlCommand (UPDATED):" << std::endl;
                    std::cout << "  Sequence: " << cmd_seq << std::endl;
                    std::cout << "  Time: " << control_command_msg.stamp.sec << "." 
                              << std::setw(9) << std::setfill('0') 
                              << control_command_msg.stamp.nanosec << std::endl;
                    printControlCommand(control_command_msg);
                } else {
                    std::cout << "VehicleControlCommand (no update)" << std::endl;
                }
            } else {
                std::cout << "VehicleControlCommand: READ FAILED" << std::endl;
            }
            
            // Read trajectory
            uint64_t new_traj_seq;
            bool success_traj = reader.read(
                davos::shm::AutowareMsgType::Trajectory, 
                trajectory_msg, 
                &new_traj_seq
            );
            
            if (success_traj) {
                if (new_traj_seq != traj_seq) {
                    traj_seq = new_traj_seq;
                    std::cout << "Trajectory (UPDATED):" << std::endl;
                    std::cout << "  Sequence: " << traj_seq << std::endl;
                    std::cout << "  Frame ID: " << trajectory_msg.header.frame_id << std::endl;
                    printTrajectory(trajectory_msg);
                } else {
                    std::cout << "Trajectory (no update)" << std::endl;
                }
            } else {
                std::cout << "Trajectory: READ FAILED" << std::endl;
            }
            
            // Sleep for a while
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        
        std::cout << "Reader shutting down..." << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
