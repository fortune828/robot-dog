"""ground_filter_node.py — 3D coordinate transform + spatial filter.

Optional debug node for filtering the DA3 wrapper PointCloud2 output.

Two filtering modes (select via `filter_mode` param):
  "pass_through" — simple Z-axis crop (fast, best for flat ground)
  "bev_adaptive" — per-cell min-Z ground estimation (handles slopes)

Also publishes the missing static TF base_link → camera_link so the TF tree
is complete for Nav2 sensor transforms.

Transform chain:
    1. Convert DA3 optical coordinates when input_convention=optical
    2. Camera → Base link: pitch rotation around Y
    3. Spatial filter in base_link frame: X > blind_spot & points above ground
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

try:
    from tf2_ros import StaticTransformBroadcaster
    from geometry_msgs.msg import TransformStamped
    HAS_TF = True
except ImportError:
    HAS_TF = False


class GroundFilterNode(Node):
    """3D transform + spatial filter: camera → base_link → crop."""

    def __init__(self):
        super().__init__("ground_filter_node")

        # ---- parameters ----
        self.declare_parameter("input_topic", "/depth_anything_v3/output/point_cloud")
        self.declare_parameter("input_convention", "optical")
        self.declare_parameter("output_topic", "/depth_anything/points_filtered")
        self.declare_parameter("filter_mode", "bev_adaptive")
        self.declare_parameter("min_z", 0.08)     # 与 YAML 一致
        self.declare_parameter("max_z", 1.50)     # 与 YAML 一致
        self.declare_parameter("output_frame", "camera_link")
        self.declare_parameter("blind_spot", 2.0) # 与 YAML 一致
        self.declare_parameter("camera_height", 1.0) # 与 YAML 一致: 相机物理安装高度
        self.declare_parameter("camera_pitch_deg", 0.0) # 与 YAML 一致
        self.declare_parameter("log_interval", 30)
        self.declare_parameter("bev_resolution", 0.15)   # 与 YAML 一致
        self.declare_parameter("bev_height_diff", 0.12) # 与 YAML 一致
        self.declare_parameter("camera_x", 0.15)
        self.declare_parameter("camera_y", 0.0)
        self.declare_parameter("camera_z", 1.0)
        self.declare_parameter("camera_roll", 0.0)
        self.declare_parameter("camera_pitch", 0.0)
        self.declare_parameter("camera_yaw", 0.0)

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        self._input_convention = self.get_parameter("input_convention").get_parameter_value().string_value.lower()
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self._filter_mode = self.get_parameter("filter_mode").get_parameter_value().string_value
        self._min_z = self.get_parameter("min_z").get_parameter_value().double_value
        self._max_z = self.get_parameter("max_z").get_parameter_value().double_value
        self._output_frame = self.get_parameter("output_frame").get_parameter_value().string_value
        self._blind_spot = self.get_parameter("blind_spot").get_parameter_value().double_value
        configured_height = self.get_parameter("camera_height").get_parameter_value().double_value
        self._camera_x = self.get_parameter("camera_x").get_parameter_value().double_value
        self._camera_height = self.get_parameter("camera_z").get_parameter_value().double_value
        _pitch_deg = self.get_parameter("camera_pitch_deg").get_parameter_value().double_value
        configured_pitch = self.get_parameter("camera_pitch").get_parameter_value().double_value
        self._log_interval = self.get_parameter("log_interval").get_parameter_value().integer_value
        self._bev_res = self.get_parameter("bev_resolution").get_parameter_value().double_value
        self._bev_diff = self.get_parameter("bev_height_diff").get_parameter_value().double_value
        if self._input_convention not in ("standard", "optical"):
            raise ValueError("input_convention must be standard or optical")

        # precompute pitch rotation trig
        import math
        self._camera_pitch_rad = configured_pitch if abs(configured_pitch) > 1e-9 else math.radians(_pitch_deg)
        if abs(configured_height - self._camera_height) > 1e-6:
            self.get_logger().warn("camera_height differs from camera_z; using camera_z for filtering and TF")
        self._cos_pitch = math.cos(self._camera_pitch_rad)
        self._sin_pitch = math.sin(self._camera_pitch_rad)

        # ---- static TF: base_link → camera_link ----
        self._publish_static_tf()

        # ---- pub/sub ----
        self._sub = self.create_subscription(
            PointCloud2, input_topic, self._cloud_callback, 10
        )
        self._pub = self.create_publisher(PointCloud2, output_topic, 10)

        self._frame_seq = 0

        self.get_logger().info(
            f"GroundFilter ready | {input_topic} → {output_topic} | "
            f"mode={self._filter_mode} | input={self._input_convention} | "
            f"h={self._camera_height:.2f}m pitch={_pitch_deg:.1f}° | "
            f"X>{self._blind_spot:.2f}m "
            f"Z∈[{self._min_z:.2f}, {self._max_z:.2f}]m | "
            f"frame={self._output_frame}"
        )

    # ------------------------------------------------------------------
    #  Static TF
    # ------------------------------------------------------------------

    def _publish_static_tf(self):
        """Publish base_link → camera_link static transform.

        Camera mounted 1.0m above ground, 15cm forward of base_link.
        This TF is what Nav2 uses to transform point cloud Z-coordinates
        from camera frame (origin at lens) to base_link frame (origin on ground).
        """
        if not HAS_TF:
            self.get_logger().warn("tf2_ros unavailable, skipping static TF")
            return

        import math
        cx = self._camera_x
        cy = self.get_parameter("camera_y").get_parameter_value().double_value
        cz = self._camera_height
        roll = self.get_parameter("camera_roll").get_parameter_value().double_value
        pitch = self._camera_pitch_rad
        yaw = self.get_parameter("camera_yaw").get_parameter_value().double_value

        tf_broadcaster = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "camera_link"
        t.transform.translation.x = cx
        t.transform.translation.y = cy
        t.transform.translation.z = cz

        # Euler → quaternion (ZYX)
        cy_r = math.cos(yaw / 2.0)
        sy_r = math.sin(yaw / 2.0)
        cp = math.cos(pitch / 2.0)
        sp = math.sin(pitch / 2.0)
        cr = math.cos(roll / 2.0)
        sr = math.sin(roll / 2.0)
        t.transform.rotation.w = cr * cp * cy_r + sr * sp * sy_r
        t.transform.rotation.x = sr * cp * cy_r - cr * sp * sy_r
        t.transform.rotation.y = cr * sp * cy_r + sr * cp * sy_r
        t.transform.rotation.z = cr * cp * sy_r - sr * sp * cy_r

        tf_broadcaster.sendTransform(t)
        self.get_logger().info(
            f"Static TF published: base_link → camera_link "
            f"({cx:.2f}, {cy:.2f}, {cz:.2f})"
        )

    # ------------------------------------------------------------------
    #  PointCloud callback — spatial filter only, TF handles transforms
    # ------------------------------------------------------------------

    def _cloud_callback(self, msg: PointCloud2):
        n_points = msg.width * msg.height if msg.height > 1 else msg.width
        if n_points == 0:
            return

        if msg.is_bigendian or msg.point_step < 12 or msg.point_step % 4 != 0 or len(msg.data) % msg.point_step != 0:
            self.get_logger().error("Unsupported PointCloud2 layout; expected little-endian float32 XYZ[+field]")
            return

        floats_per_point = msg.point_step // 4
        data = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, floats_per_point)
        if self._input_convention == "optical":
            # ROS optical: X right, Y down, Z forward -> standard: X forward, Y left, Z up.
            X_cam = data[:, 2]
            Y_cam = -data[:, 0]
            Z_cam = -data[:, 1]
        else:
            X_cam = data[:, 0]
            Y_cam = data[:, 1]
            Z_cam = data[:, 2]
        intensity = data[:, 3] if floats_per_point >= 4 else X_cam

        # Transform to base_link coordinates for filtering; output stays in camera_link.
        X_base = self._cos_pitch * X_cam + self._sin_pitch * Z_cam + self._camera_x
        Y_base = Y_cam
        Z_base = -self._sin_pitch * X_cam + self._cos_pitch * Z_cam + self._camera_height

        # ---- blind spot and invalid-point filter ----
        finite_mask = np.isfinite(X_base) & np.isfinite(Y_base) & np.isfinite(Z_base)
        fwd_mask = finite_mask & (X_base > self._blind_spot)

        # ---- ground filter (mode-dependent) ----
        if self._filter_mode == "bev_adaptive":
            height_mask = self._bev_height_filter(X_base, Y_base, Z_base, fwd_mask)
        else:
            # pass_through: 障碍物比地面高 min_z, 绝对高度不超过 max_z
            height_mask = (Z_base >= self._min_z) & (Z_base <= self._max_z)

        mask = fwd_mask & height_mask

        self._frame_seq += 1

        # ---- repack ----
        filtered = np.column_stack([
            X_cam[mask], Y_cam[mask], Z_cam[mask], intensity[mask]
        ]).astype(np.float32)

        out = PointCloud2()
        out.header = Header(stamp=msg.header.stamp, frame_id=self._output_frame)
        out.height = 1
        out.width = filtered.shape[0]
        out.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        out.point_step = 16
        out.row_step = 16 * filtered.shape[0]
        out.is_bigendian = False
        out.is_dense = True
        out.data = filtered.tobytes()
        self._pub.publish(out)

        # ---- stats ----
        if self._log_interval > 0 and self._frame_seq % self._log_interval == 0:
            kept_pct = 100.0 * filtered.shape[0] / n_points if n_points else 0
            self.get_logger().info(
                f"Frame #{self._frame_seq} | "
                f"in={n_points} → out={filtered.shape[0]} ({kept_pct:.1f}%) | "
                f"X>{self._blind_spot:.2f}m"
            )

    # ------------------------------------------------------------------
    #  BEV adaptive ground filter — fully vectorised per-cell min-Z
    # ------------------------------------------------------------------

    def _bev_height_filter(self, X, Y, Z, valid_mask):
        """Estimate a local ground level in each BEV cell and retain raised points."""
        result = np.zeros(len(Z), dtype=bool)
        valid_indices = np.flatnonzero(valid_mask)
        if valid_indices.size == 0:
            return result
        if self._bev_res <= 0.0:
            return result

        Xv, Yv, Zv = X[valid_mask], Y[valid_mask], Z[valid_mask]
        cells = np.column_stack((
            np.floor(Xv / self._bev_res).astype(np.int64),
            np.floor(Yv / self._bev_res).astype(np.int64),
        ))
        _, inverse = np.unique(cells, axis=0, return_inverse=True)
        local_ground = np.full(inverse.max() + 1, np.inf, dtype=np.float32)
        np.minimum.at(local_ground, inverse, Zv)
        ground_for_point = local_ground[inverse]

        min_height = max(self._min_z, self._bev_diff)
        is_obstacle = (Zv - ground_for_point) >= min_height
        is_obstacle &= (Zv - ground_for_point) <= self._max_z
        result[valid_indices] = is_obstacle
        return result


def main(args=None):
    rclpy.init(args=args)
    node = GroundFilterNode()
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
