# Troubleshooting Guide

Common issues, failure diagnostics, and remediation procedures for the decentralized AMR fleet.

---

| Symptom / Error | Root Cause | Solution |
| :--- | :--- | :--- |
| **Robots do not move; `cmd_vel` has 0 subscribers** | `diff_drive` plugin failed to load or entity was removed/unspawned | Inspect URDF `<gazebo>` plugin block. Confirm `gazebo_ros_diff_drive.so` has `<ros><remapping>cmd_vel:=cmd_vel</remapping></ros>`. Check `/get_model_list` to verify model is present. |
| **Robots pass through warehouse shelves or each other** | Shelves lack `<collision>` or ODE solver step size is too large (tunneling) | Ensure shelves have `<static>true</static>` and explicit `<collision>` box with high friction. Set ODE `<max_step_size>0.001</max_step_size>` and `<real_time_update_rate>1000</real_time_update_rate>` in `warehouse.world`. |
| **Robots spin in place or skid uncontrollably** | Wheel friction $\mu_1, \mu_2$ missing or passive casters act as brakes | Add `<gazebo reference="left_wheel">` with $\mu_1=\mu_2=1.0$, $k_p=1\times 10^6, k_d=100$. Add `<gazebo reference="front_caster">` with low friction $\mu_1=\mu_2=0.001$. |
| **`timeout 3 ros2 topic echo /robot_1/odom` outputs nothing** | Gazebo physics is paused | Call `ros2 service call /unpause_physics std_srvs/srv/Empty "{}"` or resume from the dashboard control panel. |
| **Deadlock between opposing robots in central corridor** | Head-on reciprocal obstacle infeasible for ORCA alone | ORCA safe-stop will halt the robot; `conflict_resolver` monitors stall $>3\text{s}$, computes composite priority (distance + urgency + battery), and issues a replan detour to the lower-priority robot. |
| **Dashboard shows "Disconnected" or WebSocket error** | Rosbridge not running or port 9090 blocked | Ensure `docker compose up -d rosbridge` is active and bound to the shared network namespace (`network_mode: "service:simulation"`). |
| **`colcon build` fails on custom messages** | Python install directory colliding with CMake target | Use `rosidl_generate_interfaces` with explicit dependencies and separate `ament_get_python_install_dir` directory install. |
| **Unit tests fail on host with `No module named 'rclpy'`** | Host environment lacks ROS 2 Humble packages | All nodes in `amr_fleet` include graceful `try/except ImportError` fallbacks. Core logic (`ORCAPlanner`, `AStarPlanner`, `TaskAllocator`, `ConflictResolver`, `BatteryMonitor`) is 100% pure Python and testable without ROS 2. |
