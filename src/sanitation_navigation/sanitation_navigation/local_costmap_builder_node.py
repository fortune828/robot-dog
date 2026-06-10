"""Build a small inflated BEV OccupancyGrid from filtered obstacle points."""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray

from sanitation_core.local_planning_utils import inflate_occupied

try:
    from tf2_ros import Buffer, TransformException, TransformListener
    HAS_TF = True
except ImportError:
    HAS_TF = False


class LocalCostmapBuilderNode(Node):
    def __init__(self):
        super().__init__("local_costmap_builder_node")
        defaults = {
            "cloud_topic": "/depth_anything/points_filtered",
            "grid_topic": "/local_occupancy_grid",
            "debug_image_topic": "/local_costmap_debug_image",
            "markers_topic": "/local_obstacle_markers",
            "frame_id": "base_link",
            "x_min": 0.0, "x_max": 12.0, "y_min": -5.0, "y_max": 5.0,
            "resolution": 0.1, "min_obstacle_height": 0.08, "max_obstacle_height": 1.5,
            "min_points_per_cell": 1, "robot_radius": 0.35, "safety_margin": 0.35,
            "inflation_radius": 0.7, "log_interval": 10,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._p = {name: self.get_parameter(name).value for name in defaults}
        if self._p["resolution"] <= 0 or self._p["x_max"] <= self._p["x_min"] or self._p["y_max"] <= self._p["y_min"]:
            raise ValueError("invalid local costmap bounds or resolution")
        self._width = int(math.ceil((self._p["x_max"] - self._p["x_min"]) / self._p["resolution"]))
        self._height = int(math.ceil((self._p["y_max"] - self._p["y_min"]) / self._p["resolution"]))
        configured_inflation = self._p["inflation_radius"]
        self._inflation_radius = configured_inflation if configured_inflation >= 0.0 else self._p["robot_radius"] + self._p["safety_margin"]
        self._inflation_cells = int(math.ceil(self._inflation_radius / self._p["resolution"]))
        self._grid_pub = self.create_publisher(OccupancyGrid, self._p["grid_topic"], 10)
        self._debug_pub = self.create_publisher(Image, self._p["debug_image_topic"], 10)
        self._marker_pub = self.create_publisher(MarkerArray, self._p["markers_topic"], 10)
        self._sub = self.create_subscription(PointCloud2, self._p["cloud_topic"], self._cloud_callback, 10)
        self._tf_buffer = Buffer() if HAS_TF else None
        self._tf_listener = TransformListener(self._tf_buffer, self) if HAS_TF else None
        self._seq = 0
        self.get_logger().info(f"Local costmap ready | {self._p['cloud_topic']} -> {self._p['grid_topic']} | {self._width}x{self._height}")

    def _cloud_callback(self, msg: PointCloud2):
        points = self._read_xyz(msg)
        if points is None:
            return
        points = self._transform_points(points, msg.header.frame_id)
        if points is None:
            return
        finite = np.isfinite(points).all(axis=1)
        height = (points[:, 2] >= self._p["min_obstacle_height"]) & (points[:, 2] <= self._p["max_obstacle_height"])
        bounds = ((points[:, 0] >= self._p["x_min"]) & (points[:, 0] < self._p["x_max"]) &
                  (points[:, 1] >= self._p["y_min"]) & (points[:, 1] < self._p["y_max"]))
        valid = points[finite & height & bounds]
        counts = np.zeros((self._height, self._width), dtype=np.int32)
        if valid.size:
            cols = ((valid[:, 0] - self._p["x_min"]) / self._p["resolution"]).astype(np.int32)
            rows = ((valid[:, 1] - self._p["y_min"]) / self._p["resolution"]).astype(np.int32)
            np.add.at(counts, (rows, cols), 1)
        occupied = counts >= max(1, int(self._p["min_points_per_cell"]))
        inflated = inflate_occupied(occupied, self._inflation_cells)
        self._publish_grid(msg, inflated)
        self._publish_debug(msg, occupied, inflated)
        self._publish_markers(msg, occupied, inflated)
        self._seq += 1
        if self._p["log_interval"] > 0 and self._seq % self._p["log_interval"] == 0:
            self.get_logger().info(f"costmap frame#{self._seq}: valid_points={len(valid)} occupied={occupied.sum()} inflated={inflated.sum()}")

    def _read_xyz(self, msg: PointCloud2):
        field_messages = {field.name: field for field in msg.fields}
        fields = {name: field.offset for name, field in field_messages.items()}
        xyz_float32 = all(field_messages.get(name) is not None and field_messages[name].datatype == PointField.FLOAT32 for name in ("x", "y", "z"))
        if msg.is_bigendian or msg.point_step < 12 or not xyz_float32:
            self.get_logger().warn("Unsupported PointCloud2 layout; expected little-endian float32 x/y/z")
            return None
        count = msg.width * msg.height
        if count == 0:
            return np.empty((0, 3), dtype=np.float32)
        dtype = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4", "<f4", "<f4"],
                          "offsets": [fields["x"], fields["y"], fields["z"]], "itemsize": msg.point_step})
        try:
            data = np.frombuffer(msg.data, dtype=dtype, count=count)
        except ValueError:
            self.get_logger().warn("PointCloud2 data is shorter than its declared dimensions")
            return None
        return np.column_stack((data["x"], data["y"], data["z"])).astype(np.float32, copy=False)

    def _transform_points(self, points: np.ndarray, source_frame: str):
        target = self._p["frame_id"]
        if not source_frame or source_frame == target:
            return points
        if self._tf_buffer is None:
            self.get_logger().warn(f"TF unavailable; cannot transform {source_frame} -> {target}")
            return None
        try:
            tf = self._tf_buffer.lookup_transform(target, source_frame, Time(), timeout=Duration(seconds=0.1)).transform
        except TransformException as exc:
            self.get_logger().warn(f"Waiting for TF {source_frame} -> {target}: {exc}", throttle_duration_sec=2.0)
            return None
        q = tf.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        rotation = np.array([
            [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
            [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
            [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
        ], dtype=np.float32)
        translation = np.array([tf.translation.x, tf.translation.y, tf.translation.z], dtype=np.float32)
        return points @ rotation.T + translation

    def _publish_grid(self, source, inflated):
        msg = OccupancyGrid()
        msg.header.stamp = source.header.stamp
        msg.header.frame_id = self._p["frame_id"]
        msg.info.resolution = float(self._p["resolution"])
        msg.info.width, msg.info.height = self._width, self._height
        msg.info.origin.position.x, msg.info.origin.position.y = float(self._p["x_min"]), float(self._p["y_min"])
        msg.info.origin.orientation.w = 1.0
        msg.data = np.where(inflated, 100, 0).astype(np.int8).ravel().tolist()
        self._grid_pub.publish(msg)

    def _publish_debug(self, source, occupied, inflated):
        image = np.zeros((self._height, self._width, 3), dtype=np.uint8)
        image[inflated] = (80, 80, 80)
        image[occupied] = (255, 255, 255)
        msg = Image()
        msg.header.stamp, msg.header.frame_id = source.header.stamp, self._p["frame_id"]
        msg.height, msg.width, msg.encoding = self._height, self._width, "rgb8"
        msg.step, msg.data = self._width * 3, np.flipud(image).tobytes()
        self._debug_pub.publish(msg)

    def _publish_markers(self, source, occupied, inflated):
        result = MarkerArray()
        for marker_id, (mask, color, height) in enumerate(((inflated & ~occupied, (1.0, 0.7, 0.0), 0.02), (occupied, (1.0, 0.1, 0.1), 0.06))):
            marker = Marker()
            marker.header.stamp, marker.header.frame_id = source.header.stamp, self._p["frame_id"]
            marker.ns, marker.id, marker.type, marker.action = "local_obstacles", marker_id, Marker.CUBE_LIST, Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = float(self._p["resolution"])
            marker.scale.z = height
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = *color, 0.65
            for row, col in np.argwhere(mask):
                marker.points.append(Point(x=self._p["x_min"] + (col + 0.5) * self._p["resolution"],
                                           y=self._p["y_min"] + (row + 0.5) * self._p["resolution"], z=height / 2.0))
            result.markers.append(marker)
        self._marker_pub.publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = LocalCostmapBuilderNode()
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
