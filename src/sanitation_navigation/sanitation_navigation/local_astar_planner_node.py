"""Plan a clearance-aware, stable local path from each OccupancyGrid."""

import math
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from sanitation_core.local_planning_utils import (
    cell_to_world,
    early_avoidance_candidates,
    forward_corridor_has_obstacle,
    nearest_free,
    obstacle_cost_field,
    path_distance_field,
    simplify_collinear,
    weighted_astar_grid,
    world_to_cell,
)


class LocalAstarPlannerNode(Node):
    def __init__(self):
        super().__init__("local_astar_planner_node")
        defaults = {
            "grid_topic": "/local_occupancy_grid", "path_topic": "/local_path",
            "goal_marker_topic": "/local_goal_marker", "status_topic": "/planning_status",
            "start_x": 0.0, "start_y": 0.0, "goal_x": 5.0, "goal_y": 0.0,
            "allow_diagonal": True, "obstacle_threshold": 80,
            "unknown_as_obstacle": False, "path_smoothing": True,
            "early_avoidance_enabled": True, "early_avoidance_distance": 3.5,
            "early_avoidance_width": 1.0, "preferred_clearance": 1.0,
            "heuristic_weight": 2.0, "obstacle_cost_weight": 3.5,
            "smoothness_weight": 0.4, "path_change_weight": 1.0,
            "goal_direction_weight": 0.2,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._p = {name: self.get_parameter(name).value for name in defaults}
        self._path_pub = self.create_publisher(Path, self._p["path_topic"], 10)
        self._goal_pub = self.create_publisher(Marker, self._p["goal_marker_topic"], 10)
        self._status_pub = self.create_publisher(String, self._p["status_topic"], 10)
        self._sub = self.create_subscription(OccupancyGrid, self._p["grid_topic"], self._grid_callback, 10)
        self._previous_path = []
        self.get_logger().info(f"Cost-aware local A* ready | {self._p['grid_topic']} -> {self._p['path_topic']}")

    def _grid_callback(self, msg: OccupancyGrid):
        width, height = msg.info.width, msg.info.height
        if width == 0 or height == 0 or len(msg.data) != width * height or msg.info.resolution <= 0.0:
            self._fail(msg, "FAILED_INVALID_GRID")
            return
        values = np.asarray(msg.data, dtype=np.int16).reshape(height, width)
        blocked = values >= int(self._p["obstacle_threshold"])
        if self._p["unknown_as_obstacle"]:
            blocked |= values < 0
        origin_x, origin_y = msg.info.origin.position.x, msg.info.origin.position.y
        start = world_to_cell(self._p["start_x"], self._p["start_y"], origin_x, origin_y, msg.info.resolution)
        goal = world_to_cell(self._p["goal_x"], self._p["goal_y"], origin_x, origin_y, msg.info.resolution)
        if not (0 <= start[0] < height and 0 <= start[1] < width):
            self._fail(msg, "FAILED_START_OUTSIDE_GRID")
            return
        if not (0 <= goal[0] < height and 0 <= goal[1] < width):
            self._fail(msg, "FAILED_GOAL_OUTSIDE_GRID")
            return
        free_start = nearest_free(blocked, start)
        if free_start is None:
            self._fail(msg, "FAILED_START_BLOCKED")
            return
        free_goal = nearest_free(blocked, goal)
        if free_goal is None:
            self._fail(msg, "FAILED_GOAL_BLOCKED")
            return

        resolution = msg.info.resolution
        preferred_clearance = max(float(self._p["preferred_clearance"]), resolution)
        traversal_cost = obstacle_cost_field(blocked, preferred_clearance, resolution)
        previous_distance = path_distance_field(blocked.shape, self._previous_path, resolution)
        path_change_cost = np.minimum(previous_distance / preferred_clearance, 1.0)
        planner_args = {
            "traversal_cost": traversal_cost,
            "allow_diagonal": bool(self._p["allow_diagonal"]),
            "heuristic_weight": max(1.0, float(self._p["heuristic_weight"])),
            "obstacle_cost_weight": max(0.0, float(self._p["obstacle_cost_weight"])),
            "smoothness_weight": max(0.0, float(self._p["smoothness_weight"])),
            "goal_direction_weight": max(0.0, float(self._p["goal_direction_weight"])),
            "path_change_cost": path_change_cost,
            "path_change_weight": max(0.0, float(self._p["path_change_weight"])),
        }
        cells, avoidance_side = self._plan(blocked, free_start, free_goal, resolution, planner_args)
        if not cells:
            status = "STOPPED_BOTH_SIDES_BLOCKED" if avoidance_side == "blocked" else "FAILED_NO_PATH"
            self._fail(msg, status)
            return
        self._previous_path = cells
        if self._p["path_smoothing"]:
            cells = simplify_collinear(cells)
        path = Path()
        path.header = msg.header
        for cell in cells:
            x, y = cell_to_world(cell, origin_x, origin_y, msg.info.resolution)
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self._path_pub.publish(path)
        self._publish_goal(msg, free_goal, origin_x, origin_y)
        adjusted = free_start != start or free_goal != goal
        self._status_pub.publish(
            String(data=f"SUCCESS avoidance={avoidance_side} adjusted={adjusted} points={len(cells)}")
        )

    def _plan(self, blocked, start, goal, resolution, planner_args):
        if not self._p["early_avoidance_enabled"]:
            return weighted_astar_grid(blocked, start, goal, **planner_args), "none"
        distance_cells = max(1, int(math.ceil(self._p["early_avoidance_distance"] / resolution)))
        half_width_cells = max(1, int(math.ceil(0.5 * self._p["early_avoidance_width"] / resolution)))
        clearance_cells = max(1, int(math.ceil(self._p["preferred_clearance"] / resolution)))
        if not forward_corridor_has_obstacle(blocked, start, goal, distance_cells, half_width_cells):
            return weighted_astar_grid(blocked, start, goal, **planner_args), "none"

        candidates = early_avoidance_candidates(
            blocked, start, goal, distance_cells, half_width_cells, clearance_cells
        )
        options = []
        for side, waypoint in candidates:
            first = weighted_astar_grid(blocked, start, waypoint, **planner_args)
            second = weighted_astar_grid(blocked, waypoint, goal, **planner_args)
            if not first or not second:
                continue
            path = first + second[1:]
            rows, cols = zip(*path)
            score = len(path) + float(np.sum(planner_args["traversal_cost"][rows, cols]))
            options.append((score, side, path))
        if not options:
            return [], "blocked"
        _, side, path = min(options, key=lambda option: option[0])
        return path, side

    def _fail(self, msg, status):
        self._previous_path = []
        path = Path()
        path.header = msg.header
        self._path_pub.publish(path)
        self._status_pub.publish(String(data=status))
        self.get_logger().warn(status, throttle_duration_sec=2.0)

    def _publish_goal(self, grid, goal, origin_x, origin_y):
        x, y = cell_to_world(goal, origin_x, origin_y, grid.info.resolution)
        marker = Marker()
        marker.header, marker.ns, marker.id = grid.header, "local_goal", 0
        marker.type, marker.action = Marker.SPHERE, Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, y, 0.12
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.24
        marker.color.g, marker.color.a = 1.0, 1.0
        self._goal_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = LocalAstarPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
