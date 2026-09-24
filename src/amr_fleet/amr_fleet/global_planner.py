"""
Global Path Planner using A* on 2D Occupancy Grid with Rolling Horizon.
Supports dynamic obstacle inflation, blocked aisle re-routing, and rolling-horizon windowing.
"""

from __future__ import annotations
import heapq
import math
import time
from typing import List, Tuple, Set, Optional, Dict

from amr_fleet.utils import Vector2D, world_to_grid, grid_to_world


class AStarPlanner:
    """
    2D A* Planner on an Occupancy Grid.
    Designed for edge hardware (Raspberry Pi 4 / Jetson Nano) with sub-10ms plan times.
    """
    def __init__(
        self,
        grid_width: int = 60,
        grid_height: int = 40,
        resolution: float = 0.5,
        origin_x: float = -15.0,
        origin_y: float = -10.0,
        robot_radius: float = 0.35,
        safety_margin: float = 0.15
    ):
        self.width = grid_width
        self.height = grid_height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.inflation_radius_m = robot_radius + safety_margin
        self.inflation_cells = max(1, int(math.ceil(self.inflation_radius_m / resolution)))

        # 0 = free, 100 = occupied
        self.grid = [[0 for _ in range(self.height)] for _ in range(self.width)]
        self.inflated_grid = [[0 for _ in range(self.height)] for _ in range(self.width)]
        self.blocked_cells: Set[Tuple[int, int]] = set()

    def set_obstacle(self, gx: int, gy: int, occupied: bool = True) -> None:
        if 0 <= gx < self.width and 0 <= gy < self.height:
            self.grid[gx][gy] = 100 if occupied else 0
            self._update_inflation_around(gx, gy)

    def set_blocked_region(self, min_x: float, min_y: float, max_x: float, max_y: float, blocked: bool = True) -> None:
        """Mark a rectangular warehouse aisle/region as blocked or cleared."""
        gx_min, gy_min = world_to_grid(min_x, min_y, self.origin_x, self.origin_y, self.resolution)
        gx_max, gy_max = world_to_grid(max_x, max_y, self.origin_x, self.origin_y, self.resolution)

        for gx in range(max(0, gx_min), min(self.width, gx_max + 1)):
            for gy in range(max(0, gy_min), min(self.height, gy_max + 1)):
                if blocked:
                    self.blocked_cells.add((gx, gy))
                else:
                    self.blocked_cells.discard((gx, gy))

    def update_from_costmap(self, data: List[int], width: int, height: int, resolution: float, origin_x: float, origin_y: float) -> None:
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.grid = [[0 for _ in range(self.height)] for _ in range(self.width)]

        for y in range(height):
            for x in range(width):
                idx = y * width + x
                val = data[idx]
                if val >= 50 or val == -1:  # Occupied or unknown
                    self.grid[x][y] = 100

        self.recompute_all_inflation()

    def recompute_all_inflation(self) -> None:
        self.inflated_grid = [[0 for _ in range(self.height)] for _ in range(self.width)]
        r = self.inflation_cells
        r_sq = r * r

        for x in range(self.width):
            for y in range(self.height):
                if self.grid[x][y] > 0:
                    for dx in range(-r, r + 1):
                        for dy in range(-r, r + 1):
                            if dx * dx + dy * dy <= r_sq:
                                nx, ny = x + dx, y + dy
                                if 0 <= nx < self.width and 0 <= ny < self.height:
                                    self.inflated_grid[nx][ny] = 100

    def _update_inflation_around(self, gx: int, gy: int) -> None:
        r = self.inflation_cells
        r_sq = r * r
        val = self.grid[gx][gy]
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if dx * dx + dy * dy <= r_sq:
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        if val > 0:
                            self.inflated_grid[nx][ny] = 100
                        else:
                            # Recompute this cell
                            self._recheck_cell(nx, ny)

    def _recheck_cell(self, cx: int, cy: int) -> None:
        r = self.inflation_cells
        r_sq = r * r
        occupied = False
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if dx * dx + dy * dy <= r_sq:
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        if self.grid[nx][ny] > 0:
                            occupied = True
                            break
            if occupied:
                break
        self.inflated_grid[cx][cy] = 100 if occupied else 0

    def is_cell_traversable(self, gx: int, gy: int) -> bool:
        if not (0 <= gx < self.width and 0 <= gy < self.height):
            return False
        if self.inflated_grid[gx][gy] > 0:
            return False
        if (gx, gy) in self.blocked_cells:
            return False
        return True

    def is_point_traversable(self, x: float, y: float) -> bool:
        gx, gy = world_to_grid(x, y, self.origin_x, self.origin_y, self.resolution)
        return self.is_cell_traversable(gx, gy)

    def plan_path(self, start: Vector2D, goal: Vector2D) -> List[Vector2D]:
        """
        Compute optimal path from start to goal using 8-connected A*.
        Returns empty list if goal is unreachable.
        """
        start_g = world_to_grid(start.x, start.y, self.origin_x, self.origin_y, self.resolution)
        goal_g = world_to_grid(goal.x, goal.y, self.origin_x, self.origin_y, self.resolution)

        # Clamp goal if slightly out of bounds
        goal_gx = max(0, min(self.width - 1, goal_g[0]))
        goal_gy = max(0, min(self.height - 1, goal_g[1]))
        goal_g = (goal_gx, goal_gy)

        if not self.is_cell_traversable(goal_g[0], goal_g[1]):
            # Find nearest reachable neighbor to goal
            goal_g = self._find_nearest_free_cell(goal_g)
            if goal_g is None:
                return []

        # 8-connected motions with sqrt(2) diagonal cost
        motions = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)
        ]

        # Priority queue item: (f_score, g_score, (gx, gy))
        open_set: List[Tuple[float, float, Tuple[int, int]]] = []
        heapq.heappush(open_set, (self._heuristic(start_g, goal_g), 0.0, start_g))

        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        g_scores: Dict[Tuple[int, int], float] = {start_g: 0.0}
        closed_set: Set[Tuple[int, int]] = set()

        found = False
        while open_set:
            _, current_g, current = heapq.heappop(open_set)

            if current in closed_set:
                continue
            closed_set.add(current)

            if current == goal_g:
                found = True
                break

            for dx, dy, cost in motions:
                neighbor = (current[0] + dx, current[1] + dy)

                if not self.is_cell_traversable(neighbor[0], neighbor[1]):
                    continue

                # Prevent cutting through diagonal corner obstacle
                if dx != 0 and dy != 0:
                    if not self.is_cell_traversable(current[0] + dx, current[1]) or \
                       not self.is_cell_traversable(current[0], current[1] + dy):
                        continue

                tentative_g = current_g + cost * self.resolution
                if neighbor not in g_scores or tentative_g < g_scores[neighbor]:
                    g_scores[neighbor] = tentative_g
                    f = tentative_g + self._heuristic(neighbor, goal_g)
                    came_from[neighbor] = current
                    heapq.heappush(open_set, (f, tentative_g, neighbor))

        if not found:
            return []

        # Reconstruct grid path
        curr = goal_g
        grid_path = [curr]
        while curr in came_from:
            curr = came_from[curr]
            grid_path.append(curr)
        grid_path.reverse()

        # Convert to world coordinates
        world_path = [Vector2D(*grid_to_world(gx, gy, self.origin_x, self.origin_y, self.resolution)) for gx, gy in grid_path]
        
        # Smooth and inject precise start and goal endpoints
        if world_path:
            world_path[0] = Vector2D(start.x, start.y)
            world_path[-1] = Vector2D(goal.x, goal.y)

        return self._smooth_path(world_path)

    def extract_rolling_horizon(
        self,
        full_path: List[Vector2D],
        horizon_time_s: float = 6.0,
        nominal_speed_mps: float = 0.5
    ) -> List[Vector2D]:
        """
        Extract the next 5-10 second segment of the global path for rolling-horizon execution.
        """
        if not full_path:
            return []

        max_horizon_dist = horizon_time_s * nominal_speed_mps
        rolling_path: List[Vector2D] = [full_path[0]]
        accum_dist = 0.0

        for i in range(1, len(full_path)):
            segment_len = full_path[i - 1].distance_to(full_path[i])
            if accum_dist + segment_len >= max_horizon_dist:
                remain = max_horizon_dist - accum_dist
                ratio = remain / max(1e-6, segment_len)
                interp_pt = full_path[i - 1] + (full_path[i] - full_path[i - 1]) * ratio
                rolling_path.append(interp_pt)
                break
            else:
                accum_dist += segment_len
                rolling_path.append(full_path[i])

        return rolling_path

    def _heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        # Octile distance heuristic
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return (dx + dy + (1.4142 - 2.0) * min(dx, dy)) * self.resolution

    def _find_nearest_free_cell(self, target: Tuple[int, int], max_search_radius: int = 10) -> Optional[Tuple[int, int]]:
        for r in range(1, max_search_radius + 1):
            for dx in range(-r, r + 1):
                for dy in (-r, r):
                    cand = (target[0] + dx, target[1] + dy)
                    if self.is_cell_traversable(cand[0], cand[1]):
                        return cand
            for dy in range(-r + 1, r):
                for dx in (-r, r):
                    cand = (target[0] + dx, target[1] + dy)
                    if self.is_cell_traversable(cand[0], cand[1]):
                        return cand
        return None

    def _smooth_path(self, path: List[Vector2D]) -> List[Vector2D]:
        """Collinear and line-of-sight shortcutting."""
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        current_idx = 0

        while current_idx < len(path) - 1:
            furthest_idx = current_idx + 1
            for next_idx in range(len(path) - 1, current_idx + 1, -1):
                if self._line_of_sight(path[current_idx], path[next_idx]):
                    furthest_idx = next_idx
                    break
            smoothed.append(path[furthest_idx])
            current_idx = furthest_idx

        return smoothed

    def _line_of_sight(self, p1: Vector2D, p2: Vector2D) -> bool:
        dist = p1.distance_to(p2)
        steps = max(2, int(math.ceil(dist / (self.resolution * 0.5))))
        for s in range(1, steps):
            ratio = s / steps
            interp = p1 + (p2 - p1) * ratio
            if not self.is_point_traversable(interp.x, interp.y):
                return False
        return True


# ==============================================================================
# ROS 2 Global Planner Node
# ==============================================================================

def main(args=None):
    try:
        import rclpy
        from rclpy.node import Node
        from nav_msgs.msg import OccupancyGrid, Path
        from geometry_msgs.msg import PoseStamped, Point
        from std_msgs.msg import String
    except ImportError:
        print("[GlobalPlanner] rclpy not detected; run in pure Python or ROS 2 container.")
        return

    rclpy.init(args=args)

    class GlobalPlannerNode(Node):
        def __init__(self):
            super().__init__('global_planner')
            self.declare_parameter('robot_id', 'robot_1')
            self.declare_parameter('rolling_horizon_s', 8.0)
            self.declare_parameter('nominal_speed', 0.5)

            self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
            self.horizon_s = self.get_parameter('rolling_horizon_s').get_parameter_value().double_value
            self.nominal_speed = self.get_parameter('nominal_speed').get_parameter_value().double_value

            self.planner = AStarPlanner()
            self.current_pose = Vector2D(0.0, 0.0)
            self.current_goal: Optional[Vector2D] = None
            self.full_path: List[Vector2D] = []

            # Subscriptions
            self.create_subscription(OccupancyGrid, '/map', self.map_callback, 1)
            self.create_subscription(PoseStamped, f'/{self.robot_id}/pose', self.pose_callback, 10)
            self.create_subscription(Point, f'/fleet/{self.robot_id}/goal', self.goal_callback, 10)
            self.create_subscription(String, '/fleet/blocked_aisles', self.blocked_aisle_callback, 10)

            # Publishers
            self.plan_pub = self.create_publisher(Path, f'/fleet/{self.robot_id}/plan', 10)
            self.timer = self.create_timer(1.0, self.planning_timer_callback)

            self.get_logger().info(f"GlobalPlanner initialized for {self.robot_id}")

        def map_callback(self, msg: OccupancyGrid):
            data = list(msg.data)
            self.planner.update_from_costmap(
                data, msg.info.width, msg.info.height,
                msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y
            )

        def pose_callback(self, msg: PoseStamped):
            self.current_pose = Vector2D(msg.pose.position.x, msg.pose.position.y)

        def goal_callback(self, msg: Point):
            new_goal = Vector2D(msg.x, msg.y)
            if self.current_goal is None or self.current_goal.distance_to(new_goal) > 0.1:
                self.current_goal = new_goal
                self.replan()

        def blocked_aisle_callback(self, msg: String):
            # Parse payload "x_min,y_min,x_max,y_max"
            try:
                coords = [float(v.strip()) for v in msg.data.split(',')]
                if len(coords) == 4:
                    self.planner.set_blocked_region(*coords, blocked=True)
                    self.replan()
            except Exception as e:
                self.get_logger().warn(f"Failed to parse blocked aisle msg: {e}")

        def replan(self):
            if self.current_goal is None:
                return
            self.full_path = self.planner.plan_path(self.current_pose, self.current_goal)
            rolling_path = self.planner.extract_rolling_horizon(self.full_path, self.horizon_s, self.nominal_speed)
            self.publish_path(rolling_path)

        def planning_timer_callback(self):
            if self.current_goal and self.current_pose.distance_to(self.current_goal) > 0.3:
                # Rolling-horizon periodic re-plan
                self.replan()

        def publish_path(self, path_pts: List[Vector2D]):
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = "map"
            for pt in path_pts:
                ps = PoseStamped()
                ps.header = path_msg.header
                ps.pose.position.x = pt.x
                ps.pose.position.y = pt.y
                path_msg.poses.append(ps)
            self.plan_pub.publish(path_msg)

    node = GlobalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
