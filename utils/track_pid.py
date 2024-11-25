import psutil
import time
import csv

# Replace with your process ID (PID)
pid = 1591112  # Update this with the PID of the process you want to monitor
process = psutil.Process(pid)

# CSV output file
output_file = "results/process_metrics_ros2_shm_subscriber.csv"

# Open the CSV file for writing
with open(output_file, mode='w', newline='') as f:
    writer = csv.writer(f)
    # Write the header
    writer.writerow([
        "Timestamp", "RSS (MB)", "Shared (MB)", 
        "User CPU Time (s)", "System CPU Time (s)", 
        "Voluntary Context Switches", "Involuntary Context Switches"
    ])
    
    try:
        while True:
            start_time = time.time()
            
            # Memory info
            memory_info = process.memory_info()
            rss = memory_info.rss / (1024 * 1024)  # Convert RSS to MB
            shared = memory_info.shared / (1024 * 1024)  # Convert Shared memory to MB
            
            # CPU time
            cpu_times = process.cpu_times()
            user_time = cpu_times.user  # Time spent in user mode
            system_time = cpu_times.system  # Time spent in kernel mode
            
            # Context switches
            ctx_switches = process.num_ctx_switches()
            voluntary_switches = ctx_switches.voluntary
            involuntary_switches = ctx_switches.involuntary
            
            # Log data
            timestamp = time.time()
            writer.writerow([
                timestamp, rss, shared, user_time, system_time, 
                voluntary_switches, involuntary_switches
            ])
            
            # Print to console (optional)
            print(f"Time: {timestamp:.2f}, RSS: {rss:.2f} MB, Shared: {shared:.2f} MB, User Time: {user_time:.2f}s, System Time: {system_time:.2f}s, Voluntary Switches: {voluntary_switches}, Involuntary Switches: {involuntary_switches}")
            
            # Sleep for 50ms
            elapsed_time = time.time() - start_time
            time.sleep(max(0, 0.05 - elapsed_time))
    except KeyboardInterrupt:
        print("Logging stopped by user.")
    except psutil.NoSuchProcess:
        print("Process ended.")
