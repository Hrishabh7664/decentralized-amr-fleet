# ROS 2 Topics and Interfaces Specification

This document details the complete topic, service, and message topology for the decentralized AMR fleet.

---

## 1. Global Fleet Mesh Topics

| Topic | Type | QoS | Description |
| :--- | :--- | :--- | :--- |
| `/fleet/active_robots` | `std_msgs/msg/String` (JSON) | Reliable, Transient Local (depth 10) | Current active AMR list and speed scale |
| `/fleet/add_robot` | `std_msgs/msg/String` | Reliable, Volatile (depth 10) | Command trigger to dynamically spawn a new AMR |
| `/fleet/remove_robot` | `std_msgs/msg/String` | Reliable, Volatile (depth 10) | Command trigger to despawn an AMR |
| `/fleet/speed_scale` | `std_msgs/msg/Float32` | Reliable, Transient Local (depth 5) | Global velocity multiplier (0.1x to 3.0x) |
| `/fleet/bids` | `std_msgs/msg/String` | Reliable, Volatile (depth 50) | Decentralized auction bids broadcast across fleet |
| `/fleet/task_assignments` | `std_msgs/msg/String` | Reliable, Volatile (depth 50) | Awarded auction tasks with pickup points |
| `/fleet/conflicts` | `std_msgs/msg/String` | Reliable, Volatile (depth 50) | Spatiotemporal deadlock and token negotiations |
| `/fleet/charging_stations` | `std_msgs/msg/String` | Reliable, Transient Local (depth 10) | Station claim/release notifications |
| `/fleet/blocked_aisles` | `std_msgs/msg/String` | Reliable, Transient Local (depth 10) | Rectangular blocked zones for A* costmap updates |

---

## 2. Per-Robot Namespace Topics (`/fleet/{robot_id}/...`)

| Topic | Type | QoS | Description |
| :--- | :--- | :--- | :--- |
| `/fleet/{id}/state` | `amr_fleet/msg/RobotState` or `String` | Best Effort, 100ms deadline, Volatile (10-20 Hz) | Live telemetry (pose, velocity, battery, status) |
| `/fleet/{id}/intent` | `amr_fleet/msg/RobotIntent` or `String` | Reliable, Transient Local (depth 10, 2 Hz) | Planned waypoints, priority score, corridor tokens |
| `/fleet/{id}/goal` | `geometry_msgs/msg/Point` | Reliable, Volatile (depth 10) | Current target navigation destination |
| `/fleet/{id}/plan` | `nav_msgs/msg/Path` | Reliable, Volatile (depth 10, 1 Hz) | Rolling-horizon global path waypoints |
| `/fleet/{id}/battery` | `std_msgs/msg/Float32` | Reliable, Volatile (depth 10, 2 Hz) | Battery percentage reading (0.0 to 100.0%) |
| `/fleet/{id}/bid` | `std_msgs/msg/String` | Reliable, Volatile (depth 10) | Individual robot bid cost output |
| `/fleet/{id}/conflict` | `std_msgs/msg/String` | Reliable, Volatile (depth 10) | Individual deadlock broadcast triggers |
| `/fleet/{id}/status` | `std_msgs/msg/String` | Reliable, Transient Local (depth 10) | State machine mode (`IDLE`, `NAVIGATING`, `RESOLVING`) |
| `/fleet/{id}/charging_intent`| `std_msgs/msg/String` | Reliable, Transient Local (depth 10) | Autonomous critical battery docking intent |

---

## 3. Robot Hardware / Gazebo Bridge Topics (`/{robot_id}/...`)

| Topic | Type | Description |
| :--- | :--- | :--- |
| `/{id}/cmd_vel` | `geometry_msgs/msg/Twist` | Differential drive velocity command (`linear.x`, `angular.z`) |
| `/{id}/odom` | `nav_msgs/msg/Odometry` | Ground-truth odometry published by `gazebo_ros_diff_drive` |
| `/{id}/pose` | `geometry_msgs/msg/PoseStamped` | Filtered 2D pose extracted by `state_publisher` |
| `/{id}/scan` | `sensor_msgs/msg/LaserScan` | 2D LiDAR ray scan from `gazebo_ros_ray_sensor` |
| `/{id}/joint_states` | `sensor_msgs/msg/JointState` | Left and right wheel joint positions and velocities |
| `/{id}/robot_description` | `std_msgs/msg/String` | Evaluated URDF model for `robot_state_publisher` |

---

## 4. Service Endpoints

| Service | Type | Provider | Description |
| :--- | :--- | :--- | :--- |
| `/spawn_entity` | `gazebo_msgs/srv/SpawnEntity` | Gazebo Classic | Spawns AMR URDF model at runtime |
| `/delete_entity` | `gazebo_msgs/srv/DeleteEntity` | Gazebo Classic | Despawns AMR entity from world |
| `/pause_physics` | `std_srvs/srv/Empty` | Gazebo Classic | Halts physics step integration |
| `/unpause_physics` | `std_srvs/srv/Empty` | Gazebo Classic | Resumes physics step integration |
| `/gazebo/pause_physics` | `std_srvs/srv/Empty` | Fleet Controller | REST/WebSocket alias for pausing physics |
| `/gazebo/unpause_physics` | `std_srvs/srv/Empty` | Fleet Controller | REST/WebSocket alias for resuming physics |
