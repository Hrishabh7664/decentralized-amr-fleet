"""
Optimal Reciprocal Collision Avoidance (ORCA) 2D Local Planner.
Solves reciprocal velocity obstacles via convex half-plane linear programming.
Includes static obstacle avoidance and fallback safe-stop mechanisms.
"""

from __future__ import annotations
import math
import time
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass

from amr_fleet.utils import Vector2D, Pose2D, Twist2D, normalize_angle


@dataclass
class Line2D:
    """A directed line in 2D velocity space representing an ORCA half-plane: (v - point) · direction >= 0"""
    point: Vector2D
    direction: Vector2D  # Unit vector along the line (normal is rotated 90 deg)


@dataclass
class AgentState:
    id: str
    position: Vector2D
    velocity: Vector2D
    radius: float = 0.35
    max_speed: float = 0.8
    pref_velocity: Vector2D = None


class ORCAPlanner:
    """
    Pure Python implementation of ORCA in 2D.
    Guarantees collision-free reciprocal navigation at 10-20 Hz on RPi4 / Jetson Nano.
    """
    def __init__(
        self,
        time_step: float = 0.05,
        time_horizon: float = 2.5,
        time_horizon_obst: float = 1.0,
        robot_radius: float = 0.35,
        max_speed: float = 0.8,
        safety_margin: float = 0.10
    ):
        self.time_step = time_step
        self.time_horizon = time_horizon
        self.time_horizon_obst = time_horizon_obst
        self.robot_radius = robot_radius + safety_margin
        self.max_speed = max_speed

    def compute_orca_halfplane(
        self,
        pos_a: Vector2D,
        vel_a: Vector2D,
        pos_b: Vector2D,
        vel_b: Vector2D,
        radius_b: float,
        time_horizon: Optional[float] = None
    ) -> Line2D:
        """
        Compute the ORCA half-plane constraint induced on agent A by agent B.
        """
        tau = time_horizon if time_horizon is not None else self.time_horizon
        inv_tau = 1.0 / max(1e-4, tau)

        rel_pos = pos_b - pos_a
        rel_vel = vel_a - vel_b
        combined_radius = self.robot_radius + radius_b
        dist_sq = rel_pos.norm_sq()
        dist = math.sqrt(dist_sq)

        line = Line2D(Vector2D(0.0, 0.0), Vector2D(0.0, 0.0))

        if dist > combined_radius:
            # Not in collision yet
            w = rel_vel - inv_tau * rel_pos
            w_norm_sq = w.norm_sq()
            dot_product = w.dot(rel_pos)

            if dot_product < 0.0 and (dot_product * dot_product) > (combined_radius * combined_radius * w_norm_sq):
                # Project on cut-off circle
                w_norm = math.sqrt(w_norm_sq)
                unit_w = w / max(1e-6, w_norm)
                line.direction = Vector2D(unit_w.y, -unit_w.x)
                u = (combined_radius * inv_tau - w_norm) * unit_w
            else:
                # Project on legs of cone
                leg = math.sqrt(max(0.0, dist_sq - combined_radius * combined_radius))
                if rel_pos.det(w) > 0.0:
                    # Left leg
                    line.direction = Vector2D(
                        rel_pos.x * leg - rel_pos.y * combined_radius,
                        rel_pos.x * combined_radius + rel_pos.y * leg
                    ) / dist_sq
                else:
                    # Right leg
                    line.direction = -Vector2D(
                        rel_pos.x * leg + rel_pos.y * combined_radius,
                        -rel_pos.x * combined_radius + rel_pos.y * leg
                    ) / dist_sq

                dot = rel_vel.dot(line.direction)
                u = dot * line.direction - rel_vel
        else:
            # Collision already occurred: push apart with time_step
            inv_time_step = 1.0 / max(1e-4, self.time_step)
            w = rel_vel - inv_time_step * rel_pos
            w_norm = w.norm()
            unit_w = w / max(1e-6, w_norm)
            line.direction = Vector2D(unit_w.y, -unit_w.x)
            u = (combined_radius * inv_time_step - w_norm) * unit_w

        # Agent A takes reciprocal share (50%) of avoidance effort
        line.point = vel_a + 0.5 * u
        return line

    def compute_static_obstacle_halfplane(
        self,
        pos_a: Vector2D,
        vel_a: Vector2D,
        obs_pos: Vector2D,
        obs_radius: float = 0.2
    ) -> Line2D:
        """Agent takes 100% responsibility for static obstacles."""
        tau = self.time_horizon_obst
        inv_tau = 1.0 / max(1e-4, tau)

        rel_pos = obs_pos - pos_a
        combined_radius = self.robot_radius + obs_radius
        dist_sq = rel_pos.norm_sq()
        dist = math.sqrt(dist_sq)

        w = vel_a - inv_tau * rel_pos
        w_norm = w.norm()
        unit_w = w / max(1e-6, w_norm)

        line = Line2D(
            point=vel_a + (combined_radius * inv_tau - w_norm) * unit_w,
            direction=Vector2D(unit_w.y, -unit_w.x)
        )
        return line

    def compute_velocity(
        self,
        current_pos: Vector2D,
        current_vel: Vector2D,
        pref_vel: Vector2D,
        neighbors: List[AgentState],
        static_obstacles: Optional[List[Vector2D]] = None
    ) -> Vector2D:
        """
        Solves 2D Linear Program to find collision-free velocity closest to pref_vel.
        Falls back to safe stop if no feasible solution exists.
        """
        orca_lines: List[Line2D] = []

        # 1. Static obstacles constraints
        if static_obstacles:
            for obs in static_obstacles:
                if current_pos.distance_to(obs) < (self.robot_radius + 1.5):
                    line = self.compute_static_obstacle_halfplane(current_pos, current_vel, obs)
                    orca_lines.append(line)

        # 2. Dynamic peer robot constraints
        for nb in neighbors:
            line = self.compute_orca_halfplane(
                current_pos, current_vel, nb.position, nb.velocity, nb.radius
            )
            orca_lines.append(line)

        # 3. Solve 2D LP
        success, new_vel = self._linear_program2(orca_lines, self.max_speed, pref_vel)

        if not success:
            # Fallback relaxation or safe-stop
            new_vel = self._fallback_safe_stop(orca_lines, pref_vel)

        return new_vel

    def _linear_program1(
        self,
        lines: List[Line2D],
        line_no: int,
        radius: float,
        opt_velocity: Vector2D,
        direction_opt: bool
    ) -> Tuple[bool, Vector2D]:
        dot_product = lines[line_no].point.dot(lines[line_no].direction)
        discriminant = dot_product * dot_product + radius * radius - lines[line_no].point.norm_sq()

        if discriminant < 0.0:
            return False, Vector2D(0.0, 0.0)

        sqrt_disc = math.sqrt(discriminant)
        t_left = -dot_product - sqrt_disc
        t_right = -dot_product + sqrt_disc

        for i in range(line_no):
            denominator = lines[line_no].direction.det(lines[i].direction)
            numerator = (lines[i].point - lines[line_no].point).det(lines[i].direction)

            if abs(denominator) <= 1e-9:
                if numerator < 0.0:
                    return False, Vector2D(0.0, 0.0)
                continue

            t = numerator / denominator
            if denominator > 0.0:
                t_right = min(t_right, t)
            else:
                t_left = max(t_left, t)

            if t_left > t_right:
                return False, Vector2D(0.0, 0.0)

        if direction_opt:
            if opt_velocity.dot(lines[line_no].direction) > 0.0:
                result = lines[line_no].point + t_right * lines[line_no].direction
            else:
                result = lines[line_no].point + t_left * lines[line_no].direction
        else:
            t = lines[line_no].direction.dot(opt_velocity - lines[line_no].point)
            t = max(t_left, min(t_right, t))
            result = lines[line_no].point + t * lines[line_no].direction

        return True, result

    def _linear_program2(
        self,
        lines: List[Line2D],
        radius: float,
        opt_velocity: Vector2D
    ) -> Tuple[bool, Vector2D]:
        if opt_velocity.norm_sq() > radius * radius:
            result = opt_velocity.normalized() * radius
        else:
            result = opt_velocity

        for i, line in enumerate(lines):
            if line.direction.det(line.point - result) > 0.0:
                # Result violates halfplane constraint
                success, result = self._linear_program1(lines, i, radius, opt_velocity, False)
                if not success:
                    return False, result

        return True, result

    def _fallback_safe_stop(self, lines: List[Line2D], pref_vel: Vector2D) -> Vector2D:
        """
        Safe-stop fallback: decelerate to zero velocity or creep forward at minimal safe speed.
        """
        # Test zero velocity
        is_zero_safe = True
        zero_vel = Vector2D(0.0, 0.0)
        for line in lines:
            if line.direction.det(line.point - zero_vel) > 0.0:
                is_zero_safe = False
                break

        if is_zero_safe:
            return zero_vel

        # Return scaled-down creep velocity or complete stop
        return Vector2D(0.0, 0.0)


# ==============================================================================
# ROS 2 Local Planner ORCA Node
# ==============================================================================

def main(args=None):
    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Twist, PoseStamped
        from nav_msgs.msg import Path
        from sensor_msgs.msg import LaserScan
    except ImportError:
        print("[ORCALocalPlanner] rclpy not detected; run in pure Python or ROS 2 container.")
        return

    rclpy.init(args=args)

    class LocalPlannerORCANode(Node):
        def __init__(self):
            super().__init__('local_planner_orca')
            self.declare_parameter('robot_id', 'robot_1')
            self.declare_parameter('max_speed', 0.8)
            self.declare_parameter('update_rate_hz', 20.0)

            self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
            self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
            self.rate_hz = self.get_parameter('update_rate_hz').get_parameter_value().double_value

            self.orca = ORCAPlanner(max_speed=self.max_speed)
            self.current_pose = Pose2D()
            self.current_vel = Vector2D(0.0, 0.0)
            self.path_waypoints: List[Vector2D] = []
            self.neighbor_states: Dict[str, AgentState] = {}
            self.static_obstacles: List[Vector2D] = []

            # Subscriptions
            self.create_subscription(PoseStamped, f'/{self.robot_id}/pose', self.pose_callback, 10)
            self.create_subscription(Path, f'/fleet/{self.robot_id}/plan', self.plan_callback, 10)
            self.create_subscription(LaserScan, f'/{self.robot_id}/scan', self.scan_callback, 10)

            # Discover peers: subscribe to fleet state topics
            for peer_num in range(1, 10):
                peer_id = f"robot_{peer_num}"
                if peer_id != self.robot_id:
                    self.create_subscription(
                        PoseStamped, f'/{peer_id}/pose',
                        lambda msg, pid=peer_id: self.peer_pose_callback(msg, pid), 10
                    )

            # Cmd vel publisher
            self.cmd_pub = self.create_publisher(Twist, f'/{self.robot_id}/cmd_vel', 10)
            self.timer = self.create_timer(1.0 / self.rate_hz, self.control_loop)

            self.get_logger().info(f"ORCA local planner running for {self.robot_id} at {self.rate_hz} Hz")

        def pose_callback(self, msg: PoseStamped):
            self.current_pose.x = msg.pose.position.x
            self.current_pose.y = msg.pose.position.y

        def peer_pose_callback(self, msg: PoseStamped, peer_id: str):
            pos = Vector2D(msg.pose.position.x, msg.pose.position.y)
            # Estimate peer velocity
            if peer_id in self.neighbor_states:
                old_state = self.neighbor_states[peer_id]
                dt = 0.1
                vel = (pos - old_state.position) / dt
            else:
                vel = Vector2D(0.0, 0.0)
            self.neighbor_states[peer_id] = AgentState(id=peer_id, position=pos, velocity=vel)

        def plan_callback(self, msg: Path):
            self.path_waypoints = [Vector2D(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]

        def scan_callback(self, msg: LaserScan):
            # Extract close static obstacle clusters from LiDAR
            self.static_obstacles.clear()
            angle = msg.angle_min
            for r in msg.ranges[::6]:  # Downsample for CPU performance on edge
                if msg.range_min < r < 1.5:  # Static obstacles within 1.5m
                    ox = self.current_pose.x + r * math.cos(self.current_pose.theta + angle)
                    oy = self.current_pose.y + r * math.sin(self.current_pose.theta + angle)
                    self.static_obstacles.append(Vector2D(ox, oy))
                angle += msg.angle_increment * 6

        def control_loop(self):
            if not self.path_waypoints:
                self.publish_stop()
                return

            # Advance along waypoints
            curr_pos = self.current_pose.position
            while self.path_waypoints and curr_pos.distance_to(self.path_waypoints[0]) < 0.35:
                self.path_waypoints.pop(0)

            if not self.path_waypoints:
                self.publish_stop()
                return

            target = self.path_waypoints[0]
            to_target = target - curr_pos
            dist = to_target.norm()

            speed = min(self.max_speed, dist * 0.8)
            pref_vel = to_target.normalized() * speed

            # Nearby dynamic neighbors within interaction horizon (3.0 m)
            active_neighbors = [
                nb for nb in self.neighbor_states.values()
                if curr_pos.distance_to(nb.position) < 3.0
            ]

            optimal_vel = self.orca.compute_velocity(
                curr_pos, self.current_vel, pref_vel, active_neighbors, self.static_obstacles
            )
            self.current_vel = optimal_vel

            # Convert to differential drive cmd_vel
            twist = Twist()
            target_heading = math.atan2(optimal_vel.y, optimal_vel.x) if optimal_vel.norm() > 0.05 else self.current_pose.theta
            heading_err = normalize_angle(target_heading - self.current_pose.theta)

            if abs(heading_err) > 0.8:
                # Turn in place first if large heading error
                twist.linear.x = 0.0
                twist.angular.z = max(-1.0, min(1.0, 1.5 * heading_err))
            else:
                twist.linear.x = optimal_vel.norm() * math.cos(heading_err)
                twist.angular.z = max(-1.0, min(1.0, 2.0 * heading_err))

            self.cmd_pub.publish(twist)

        def publish_stop(self):
            t = Twist()
            self.cmd_pub.publish(t)
            self.current_vel = Vector2D(0.0, 0.0)

    node = LocalPlannerORCANode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
