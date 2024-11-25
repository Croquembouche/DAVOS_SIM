# DAVOS

1. Timestamp
Description: The exact time (in seconds since the epoch) when the metrics were collected.
Use: Correlate with other logs (e.g., frame writes) to analyze system behavior over time.
2. RSS (Resident Set Size)
Description: Physical memory (RAM) currently used by the process, excluding shared memory.
Units: Megabytes (MB).
Use: Indicates how much actual RAM the process consumes, critical for understanding memory usage.
3. Shared Memory
Description: Memory shared with other processes, such as shared libraries or IPC mechanisms.
Units: Megabytes (MB).
Use: Important for analyzing shared memory utilization, particularly in systems using inter-process communication.
4. User CPU Time
Description: CPU time spent in user mode (e.g., running application-level code).
Units: Seconds.
Use: Helps understand how much of the CPU time is spent on the application's code (excluding kernel/system tasks).
5. System CPU Time
Description: CPU time spent in kernel mode (e.g., system calls, I/O operations).
Units: Seconds.
Use: Indicates how much time is spent on system-level operations, which can highlight bottlenecks in I/O or kernel interactions.
6. Voluntary Context Switches
Description: Number of times the process voluntarily gave up the CPU (e.g., waiting for I/O or synchronization).
Units: Count.
Use: High values suggest the process frequently blocks (e.g., for I/O or locks).
7. Involuntary Context Switches
Description: Number of times the process was forcibly preempted by the scheduler to allow another process to run.
Units: Count.
Use: High values indicate contention for CPU resources, possibly due to high system load.