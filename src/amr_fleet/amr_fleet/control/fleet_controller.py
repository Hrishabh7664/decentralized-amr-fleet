#!/usr/bin/env python3
"""
Fleet Controller & Mission Dispatcher Node.
Provides continuous waypoint dispatching for autonomous AMR coordination,
along with /fleet/add_robot, /fleet/remove_robot, /fleet/speed_scale,
and /gazebo/pause_physics / /gazebo/unpause_physics service proxies.
"""

from __future__ import annotations
import json
import math
import os
import subprocess
from typing import Dict, List, Optional, Tuple, Any

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Point, PoseStamped, Twist
    from std_msgs.msg import Float32, String
    from std_srvs.srv import Empty
    from gazebo_msgs.srv import SpawnEntity, DeleteEntity
    from ament_index_python.packages import get_package_share_directory
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    Node = object

    class Point:  # type: ignore
        def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None:
            self.x = x
            self.y = y
            self.z = z


DEFAULT_WAYPOINTS: Dict[str, List[Point]] = {
    'robot_1': [Point(x=8.0, y=0.0, z=0.0), Point(x=-8.0, y=0.0, z=0.0)],
    'robot_2': [Point(x=-8.0, y=0.0, z=0.0), Point(x=8.0, y=0.0, z=0.0)],
    'robot_3': [Point(x=0.0, y=-7.0, z=0.0), Point(x=0.0, y=7.0, z=0.0)],
    'robot_4': [Point(x=0.0, y=7.0, z=0.0), Point(x=0.0, y=-7.0, z=0.0)],
    'robot_5': [Point(x=6.0, y=4.0, z=0.0), Point(x=-6.0, y=4.0, z=0.0)],
}

DEFAULT_SPAWN_POSES: Dict[str, Tuple[float, float, float]] = {
    'robot_1': (-10.0, 0.0, 0.0),
    'robot_2': (10.0, 0.0, 3.14159),
    'robot_3': (0.0, 8.0, -1.57079),
    'robot_4': (0.0, -8.0, 1.57079),
    'robot_5': (-5.0, 4.0, -0.7853),
}


class FleetControllerNode(Node):  # type: ignore
    """ROS 2 Node orchestrating fleet waypoints and lifecycle services."""
    def __init__(self) -> None:
        """Initialize the fleet controller node, subscribers, publishers, and timers."""
        if not HAS_ROS2:
            raise RuntimeError("rclpy is required to instantiate FleetControllerNode.")
        super().__init__('fleet_controller')

        self.active_robots: List[str] = ['robot_1', 'robot_2', 'robot_3']
        self.robot_poses: Dict[str, Point] = {}
        self.robot_current_goal: Dict[str, Point] = {}
        self.robot_wp_indices: Dict[str, int] = {}
        self.subprocesses: Dict[str, subprocess.Popen] = {}
        self.speed_scale: float = 1.0

        # Subscriptions for robot poses
        self.pose_subs: Dict[str, Any] = {}
        for r_id in self.active_robots:
            self._subscribe_robot_pose(r_id)

        # Goal publishers
        self.goal_pubs: Dict[str, Any] = {}
        for r_id in self.active_robots:
            self._ensure_goal_publisher(r_id)

        # Dynamic management topics
        self.create_subscription(String, '/fleet/add_robot', self.add_robot_callback, 10)
        self.create_subscription(String, '/fleet/remove_robot', self.remove_robot_callback, 10)
        self.create_subscription(Float32, '/fleet/speed_scale', self.speed_scale_callback, 10)

        # Active robots publisher
        self.active_robots_pub = self.create_publisher(String, '/fleet/active_robots', 10)

        # Gazebo service clients
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self.pause_client = self.create_client(Empty, '/pause_physics')
        self.unpause_client = self.create_client(Empty, '/unpause_physics')

        # Provide aliases /gazebo/pause_physics and /gazebo/unpause_physics
        self.create_service(Empty, '/gazebo/pause_physics', self.handle_pause_physics)
        self.create_service(Empty, '/gazebo/unpause_physics', self.handle_unpause_physics)

        # Waypoint dispatch loop at 1 Hz
        self.timer = self.create_timer(1.0, self.mission_loop)
        # Broadcast active robots at 2 Hz
        self.status_timer = self.create_timer(0.5, self.broadcast_status)

        self.get_logger().info(f"FleetController initialized. Active robots: {self.active_robots}")

    def _ensure_goal_publisher(self, robot_id: str) -> None:
        """Create a goal publisher for a robot if not already created."""
        if robot_id not in self.goal_pubs:
            self.goal_pubs[robot_id] = self.create_publisher(Point, f'/fleet/{robot_id}/goal', 10)

    def _subscribe_robot_pose(self, robot_id: str) -> None:
        """Subscribe to a robot pose topic if not already subscribed."""
        if robot_id not in self.pose_subs:
            self.pose_subs[robot_id] = self.create_subscription(
                PoseStamped,
                f'/{robot_id}/pose',
                lambda msg, rid=robot_id: self.pose_callback(msg, rid),
                10
            )

    def pose_callback(self, msg: Any, robot_id: str) -> None:
        """Store the most recent pose received for a given robot."""
        self.robot_poses[robot_id] = msg.pose.position

    def speed_scale_callback(self, msg: Any) -> None:
        """Update fleet global speed scaling multiplier."""
        self.speed_scale = max(0.1, min(3.0, float(msg.data)))
        self.get_logger().info(f"Speed scale updated to: {self.speed_scale:.2f}x")

    def broadcast_status(self) -> None:
        """Broadcast JSON payload with active robots and speed scale."""
        msg = String()
        msg.data = json.dumps({
            'active_robots': self.active_robots,
            'speed_scale': self.speed_scale
        })
        self.active_robots_pub.publish(msg)

    def mission_loop(self) -> None:
        """Monitors each robot and dispatches next waypoint upon arrival."""
        for robot_id in list(self.active_robots):
            self._ensure_goal_publisher(robot_id)
            current_pos = self.robot_poses.get(robot_id)
            waypoints = DEFAULT_WAYPOINTS.get(
                robot_id,
                [Point(x=-6.0, y=0.0, z=0.0), Point(x=6.0, y=0.0, z=0.0)]
            )

            current_goal = self.robot_current_goal.get(robot_id)
            should_advance = False

            if current_goal is None:
                should_advance = True
            elif current_pos is not None:
                dist = math.hypot(current_pos.x - current_goal.x, current_pos.y - current_goal.y)
                if dist < 1.0:
                    should_advance = True

            if should_advance:
                wp_idx = self.robot_wp_indices.get(robot_id, -1) + 1
                if wp_idx >= len(waypoints):
                    wp_idx = 0
                self.robot_wp_indices[robot_id] = wp_idx
                next_goal = waypoints[wp_idx]
                self.robot_current_goal[robot_id] = next_goal
                self.goal_pubs[robot_id].publish(next_goal)
                self.get_logger().info(
                    f"Dispatched goal to {robot_id}: ({next_goal.x:.1f}, {next_goal.y:.1f})"
                )

    def await_service_future(self, future: Any, timeout_sec: float = 5.0) -> bool:
        """Wait for service future to resolve and verify success."""
        if future is None:
            return False
        import time
        start = time.time()
        while time.time() - start < timeout_sec:
            if hasattr(future, 'done') and future.done():
                res = future.result()
                return bool(getattr(res, 'success', True))
            if not hasattr(future, 'done'):
                res = getattr(future, 'result', lambda: None)()
                if res is not None:
                    return bool(getattr(res, 'success', True))
                return False
            time.sleep(0.01)
        return False

    def add_robot_callback(self, msg: Any) -> None:
        """Spawns a new robot entity in Gazebo and launches its control stack."""
        robot_name = msg.data.strip()
        if not robot_name:
            for idx in range(1, 20):
                candidate = f"robot_{idx}"
                if candidate not in self.active_robots:
                    robot_name = candidate
                    break

        if robot_name in self.active_robots:
            self.get_logger().warn(f"Robot {robot_name} is already registered!")
            return

        self.get_logger().info(f"Adding new robot: {robot_name}")
        spawn_pose = DEFAULT_SPAWN_POSES.get(robot_name, (0.0, -8.0, 1.57079))

        try:
            desc_share = get_package_share_directory('amr_description')
            xacro_file = os.path.join(desc_share, 'urdf', 'amr.urdf.xacro')
            xacro_proc = subprocess.run(
                ['xacro', xacro_file, f'robot_namespace:={robot_name}'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=True
            )
            urdf_xml = xacro_proc.stdout

            if not self.spawn_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().error("/spawn_entity service unavailable!")
                return

            req = SpawnEntity.Request()
            req.name = robot_name
            req.xml = urdf_xml
            req.robot_namespace = robot_name
            req.initial_pose.position.x = float(spawn_pose[0])
            req.initial_pose.position.y = float(spawn_pose[1])
            req.initial_pose.position.z = 0.05
            req.initial_pose.orientation.z = math.sin(spawn_pose[2] / 2.0)
            req.initial_pose.orientation.w = math.cos(spawn_pose[2] / 2.0)
            req.reference_frame = "world"

            future = self.spawn_client.call_async(req)
            if not self.await_service_future(future, timeout_sec=5.0):
                self.get_logger().error(f"Failed to spawn entity in Gazebo for {robot_name}")
                return

            # Entity verified in Gazebo; now launch onboard single-robot stack
            try:
                cmd = [
                    'ros2', 'launch', 'amr_fleet', 'single_robot.launch.py',
                    f'robot_id:={robot_name}', 'initial_battery:=85.0'
                ]
                proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
                self.subprocesses[robot_name] = proc
            except Exception as pe:
                self.get_logger().error(f"Failed to launch control stack for {robot_name}: {pe}")
                if self.delete_client.wait_for_service(timeout_sec=3.0):
                    del_req = DeleteEntity.Request()
                    del_req.name = robot_name
                    self.delete_client.call_async(del_req)
                return

            self.active_robots.append(robot_name)
            self._subscribe_robot_pose(robot_name)
            self._ensure_goal_publisher(robot_name)

            wps = DEFAULT_WAYPOINTS.get(robot_name, [Point(x=0.0, y=7.0, z=0.0)])
            self.robot_current_goal[robot_name] = wps[0]
            self.robot_wp_indices[robot_name] = 0
            self.goal_pubs[robot_name].publish(wps[0])

            self.get_logger().info(f"Successfully added and spawned {robot_name}")
            self.broadcast_status()

        except Exception as e:
            self.get_logger().error(f"Failed to add robot {robot_name}: {e}")

    def remove_robot_callback(self, msg: Any) -> None:
        """Removes an active robot from Gazebo and unregisters its stack."""
        robot_name = msg.data.strip()
        if robot_name not in self.active_robots:
            self.get_logger().warn(f"Robot {robot_name} is not in active robots list!")
            return

        self.get_logger().info(f"Removing robot: {robot_name}")
        try:
            if robot_name in self.goal_pubs:
                stop_pub = self.create_publisher(Twist, f'/{robot_name}/cmd_vel', 10)
                stop_pub.publish(Twist())

            if self.delete_client.wait_for_service(timeout_sec=3.0):
                req = DeleteEntity.Request()
                req.name = robot_name
                future = self.delete_client.call_async(req)
                if not self.await_service_future(future, timeout_sec=5.0):
                    self.get_logger().error(f"Failed to delete entity {robot_name} from Gazebo!")
                    return
            else:
                self.get_logger().error(f"/delete_entity service unavailable to remove {robot_name}!")
                return

            if robot_name in self.subprocesses:
                proc = self.subprocesses.pop(robot_name)
                proc.terminate()

            self.active_robots.remove(robot_name)
            self.robot_poses.pop(robot_name, None)
            self.robot_current_goal.pop(robot_name, None)
            self.robot_wp_indices.pop(robot_name, None)

            self.get_logger().info(f"Successfully removed {robot_name}")
            self.broadcast_status()

        except Exception as e:
            self.get_logger().error(f"Failed to remove robot {robot_name}: {e}")

    def handle_pause_physics(self, request: Any, response: Any) -> Any:
        """Proxy handler for pausing Gazebo physics simulation."""
        self.get_logger().info("Calling /pause_physics...")
        if self.pause_client.wait_for_service(timeout_sec=2.0):
            self.pause_client.call_async(Empty.Request())
        return response

    def handle_unpause_physics(self, request: Any, response: Any) -> Any:
        """Proxy handler for unpausing Gazebo physics simulation."""
        self.get_logger().info("Calling /unpause_physics...")
        if self.unpause_client.wait_for_service(timeout_sec=2.0):
            self.unpause_client.call_async(Empty.Request())
        return response


def main(args=None) -> None:
    """Entrypoint for the fleet controller node."""
    if not HAS_ROS2:
        print("[FleetController] rclpy not detected; run in ROS 2 container.")
        return
    rclpy.init(args=args)
    controller = FleetControllerNode()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        for proc in controller.subprocesses.values():
            proc.terminate()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
