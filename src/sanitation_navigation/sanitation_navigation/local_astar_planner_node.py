"""Plan a local A* path whenever a local OccupancyGrid arrives."""

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from sanitation_core.local_planning_utils import astar_grid, cell_to_world, nearest_free, simplify_collinear, world_to_cell


class LocalAstarPlannerNode(Node):
    def __init__(self):
        super().__init__("local_astar_planner_node")
        defaults = {
            "grid_topic": "/local_occupancy_grid", "path_topic": "/local_path",
            "goal_marker_topic": "/local_goal_marker", "status_topic": "/planning_status",
            "start_x": 0.0, "start_y": 0.0, "goal_x": 5.0, "goal_y": 0.0,
            "allow_diagonal": True, "obstacle_threshold": 80,
            "unknown_as_obstacle": False, "path_smoothing": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._p = {name: self.get_parameter(name).value for name in defaults}
        self._path_pub = self.create_publisher(Path, self._p["path_topic"], 10)
        self._goal_pub = self.create_publisher(Marker, self._p["goal_marker_topic"], 10)
        self._status_pub = self.create_publisher(String, self._p["status_topic"], 10)
        self._sub = self.create_subscription(OccupancyGrid, self._p["grid_topic"], self._grid_callback, 10)
        self.get_logger().info(f"Local A* ready | {self._p['grid_topic']} -> {self._p['path_topic']}")

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
        cells = astar_grid(blocked, free_start, free_goal, bool(self._p["allow_diagonal"]))
        if not cells:
            self._fail(msg, "FAILED_NO_PATH")
            return
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
        self._status_pub.publish(String(data=f"SUCCESS adjusted={adjusted} points={len(cells)}"))

    def _fail(self, msg, status):
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
