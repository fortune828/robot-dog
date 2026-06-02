"""ground_filter_node.py — 3D coordinate transform + spatial filter.

Subscribes to raw point cloud from depth_to_cloud_node.
The depth_to_cloud_node already outputs in ROS standard camera frame
(X=forward, Y=left, Z=up, frame_id=camera_link), so NO optical→camera
transform is needed here.

Two filtering modes (select via `filter_mode` param):
  "pass_through" — simple Z-axis crop (fast, best for flat ground)
  "bev_adaptive" — per-cell min-Z ground estimation (handles slopes)

Also publishes the missing static TF base_link → camera_link so the TF tree
is complete for Nav2 sensor transforms.

Transform chain:
    1. Depth-to-Cloud already outputs in camera standard frame (no step 1 needed)
    2. Camera → Base link: pitch rotation around Y (no height offset — already
       compensated in depth_to_cloud back-projection)
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
        self.declare_parameter("input_topic", "/camera/depth_points")
        self.declare_parameter("output_topic", "/depth_anything/points_filtered")
        self.declare_parameter("filter_mode", "bev_adaptive")  # "pass_through" or "bev_adaptive"
        self.declare_parameter("min_z", 0.05)     # min obstacle height above ground (m)
        self.declare_parameter("max_z", 0.80)     # max obstacle height (m)
        self.declare_parameter("output_frame", "camera_link")
        self.declare_parameter("blind_spot", 0.5)
        self.declare_parameter("camera_height", 0.0)  # ← 不再叠加! depth_to_cloud 已补偿
        self.declare_parameter("camera_pitch_deg", 5.0)
        self.declare_parameter("log_interval", 30)
        # BEV 专用参数
        self.declare_parameter("bev_resolution", 0.1)   # BEV 网格分辨率 (m)
        self.declare_parameter("bev_height_diff", 0.08) # 高度差阈值: 超过此值视为障碍物

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self._filter_mode = self.get_parameter("filter_mode").get_parameter_value().string_value
        self._min_z = self.get_parameter("min_z").get_parameter_value().double_value
        self._max_z = self.get_parameter("max_z").get_parameter_value().double_value
        self._output_frame = self.get_parameter("output_frame").get_parameter_value().string_value
        self._blind_spot = self.get_parameter("blind_spot").get_parameter_value().double_value
        self._camera_height = self.get_parameter("camera_height").get_parameter_value().double_value
        _pitch_deg = self.get_parameter("camera_pitch_deg").get_parameter_value().double_value
        self._log_interval = self.get_parameter("log_interval").get_parameter_value().integer_value
        self._bev_res = self.get_parameter("bev_resolution").get_parameter_value().double_value
        self._bev_diff = self.get_parameter("bev_height_diff").get_parameter_value().double_value

        # precompute pitch rotation trig
        import math
        _pitch_rad = math.radians(_pitch_deg)
        self._cos_pitch = math.cos(_pitch_rad)
        self._sin_pitch = math.sin(_pitch_rad)

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
            f"mode={self._filter_mode} | "
            f"h={self._camera_height:.2f}m pitch={_pitch_deg:.1f}° | "
            f"X>{self._blind_spot:.2f}m "
            f"Z∈[{self._min_z:.2f}, {self._max_z:.2f}]m | "
            f"frame={self._output_frame}"
        )

    # ------------------------------------------------------------------
    #  Static TF
    # ------------------------------------------------------------------

    def _publish_static_tf(self):
        """Publish the missing base_link → camera_link static transform.

        Default: camera mounted ~15 cm forward of base_link origin, at same
        ground-referenced height.  Both origins sit on the ground plane; the
        camera itself is at Z=0.5 m inside the camera_link frame (handled by
        depth_to_cloud_node back-projection).
        """
        if not HAS_TF:
            self.get_logger().warn("tf2_ros unavailable, skipping static TF")
            return

        self.declare_parameter("camera_x", 0.15)
        self.declare_parameter("camera_y", 0.0)
        self.declare_parameter("camera_z", 0.0)
        self.declare_parameter("camera_roll", 0.0)
        self.declare_parameter("camera_pitch", 0.0)
        self.declare_parameter("camera_yaw", 0.0)

        import math
        cx = self.get_parameter("camera_x").get_parameter_value().double_value
        cy = self.get_parameter("camera_y").get_parameter_value().double_value
        cz = self.get_parameter("camera_z").get_parameter_value().double_value
        roll = self.get_parameter("camera_roll").get_parameter_value().double_value
        pitch = self.get_parameter("camera_pitch").get_parameter_value().double_value
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
    #  PointCloud callback — 3D transform + spatial filter
    # ------------------------------------------------------------------

    def _cloud_callback(self, msg: PointCloud2):
        n_points = msg.width * msg.height if msg.height > 1 else msg.width
        if n_points == 0:
            return

        # ---- parse raw buffer → (N, 4) float32 ----
        # depth_to_cloud_node already outputs in ROS camera standard frame:
        #   X = forward, Y = left(+), Z = up(+), frame_id = "camera_link"
        # NO optical→camera conversion needed!
        data = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)
        X_cam = data[:, 0]   # forward
        Y_cam = data[:, 1]   # left(+)
        Z_cam = data[:, 2]   # up(+)
        intensity = data[:, 3]

        # ---- camera → base_link (pitch rotation around Y only) ----
        # NO height offset: depth_to_cloud already compensates camera_height
        cp = self._cos_pitch
        sp = self._sin_pitch
        X_base = X_cam * cp + Z_cam * sp
        Y_base = Y_cam
        Z_base = -X_cam * sp + Z_cam * cp

        # ---- filter by forward clearance ----
        fwd_mask = X_base > self._blind_spot

        # ---- height filter (mode-dependent) ----
        if self._filter_mode == "bev_adaptive":
            height_mask = self._bev_height_filter(X_base, Y_base, Z_base, fwd_mask)
        else:
            # pass_through: simple Z-band crop
            height_mask = (Z_base >= self._min_z) & (Z_base <= self._max_z)

        mask = fwd_mask & height_mask

        # ---- repack surviving points ----
        filtered = np.column_stack([
            X_base[mask], Y_base[mask], Z_base[mask], intensity[mask]
        ]).astype(np.float32)

        out = PointCloud2()
        out.header = Header(
            stamp=msg.header.stamp,
            frame_id=self._output_frame,
        )
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

        # ---- periodic statistics ----
        self._frame_seq += 1
        if self._log_interval > 0 and self._frame_seq % self._log_interval == 0:
            kept_pct = 100.0 * filtered.shape[0] / n_points if n_points else 0
            self.get_logger().info(
                f"Frame #{self._frame_seq} | "
                f"in={n_points} → out={filtered.shape[0]} "
                f"({kept_pct:.1f}%) | "
                f"X>{self._blind_spot:.2f}m "
                f"Z∈[{self._min_z:.2f}, {self._max_z:.2f}]m"
            )

    # ------------------------------------------------------------------
    #  BEV adaptive ground filter — fully vectorised per-cell min-Z
    # ------------------------------------------------------------------

    def _bev_height_filter(self, X, Y, Z, valid_mask):
        """BEV 自适应地面过滤（完全向量化，无 Python 循环）。

        原理:
          1. XY 平面网格化, np.minimum.at 取每个 cell 的最小 Z = 局部地面
          2. 每个点的 Z 减其 cell 的地面高度，差值 > threshold → 障碍物
          3. 地面直线约束: 对地面 cell 做加权线性拟合，过滤不符合直线趋势的噪声地面点

        复杂度: O(n)，适合 15K+ 点的实时处理
        """
        n_total = len(Z)
        if not np.any(valid_mask):
            return np.zeros(n_total, dtype=bool)

        Xv, Yv, Zv = X[valid_mask], Y[valid_mask], Z[valid_mask]

        # ---- Step 1: 网格化 ----
        xi = np.floor(Xv / self._bev_res).astype(np.int32)
        yi = np.floor(Yv / self._bev_res).astype(np.int32)
        xi -= xi.min()
        yi -= yi.min()
        grid_w = int(xi.max()) + 1
        grid_h = int(yi.max()) + 1

        # ---- Step 2: 向量化取每 cell 最小 Z（地面高度） ----
        grid_z = np.full((grid_h, grid_w), np.inf, dtype=np.float32)
        np.minimum.at(grid_z, (yi, xi), Zv)

        # ---- Step 3: 每点高度减去其 cell 的地面高度 ----
        cell_ground = grid_z[yi, xi]
        height_above_ground = Zv - cell_ground

        # ---- Step 4: 障碍物判定（高于局部地面 → 障碍物） ----
        is_obstacle = np.isfinite(cell_ground) & (height_above_ground > self._bev_diff)

        # ---- Step 5: 地面直线约束（可选，过滤远处翘起的地面噪声） ----
        # 分析地面 cell: 若某 cell 的地面高度与地面直线的偏差 > 阈值，视为噪声
        ground_cells = np.isfinite(grid_z) & (grid_z < self._max_z)
        if ground_cells.sum() > 20:
            gy, gx = np.where(ground_cells)
            gz = grid_z[gy, gx]
            # 地面直线: Z = a*X + b (随距离线性变化)
            # 加权最小二乘（近处权重更大）
            rx = gx.astype(np.float32) * self._bev_res + xi.min() * self._bev_res
            weights = 1.0 / (rx + 1.0)  # 近处权重大
            A = np.column_stack([rx, np.ones_like(rx)])
            W = np.diag(weights)
            try:
                coeffs = np.linalg.lstsq(A.T @ W @ A, A.T @ W @ gz, rcond=None)[0]
                a, b = coeffs[0], coeffs[1]
                # 对每个障碍物候选点,用地面直线验证
                expected_ground = a * Xv + b
                is_obstacle &= (Zv - expected_ground) > self._bev_diff
            except np.linalg.LinAlgError:
                pass  # 直线拟合失败，回退到纯 cell-min 判定

        # ---- Step 6: 映射回全量 mask ----
        result = np.zeros(n_total, dtype=bool)
        result[np.where(valid_mask)[0]] = is_obstacle
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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
