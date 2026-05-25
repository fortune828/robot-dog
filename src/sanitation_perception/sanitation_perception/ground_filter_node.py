"""ground_filter_node.py — 3D coordinate transform + spatial filter.

Subscribes to raw point cloud from depth_to_cloud_node (optical frame:
Z=forward, Y=down, X=right), applies a full 3D transform chain to convert
into base_link frame (X=forward, Y=left, Z=up), compensates for camera
mounting height and pitch, then filters by forward distance and obstacle
height.  Publishes the surviving points for Nav2 local costmap consumption.

Also publishes the missing static TF base_link → camera_link so the TF tree
is complete for Nav2 sensor transforms.

Transform chain (all in numpy, vectorised):
    1. Optical → Camera standard:  X_cam=Z_opt, Y_cam=-X_opt, Z_cam=-Y_opt
    2. Camera → Base link:         pitch rotation around Y + height offset
    3. Spatial crop (base_link):   X > blind_spot  &  min_z < Z < max_z
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
    """3D transform + spatial filter: optical → camera → base_link → crop."""

    def __init__(self):
        super().__init__("ground_filter_node")

        # ---- parameters ----
        self.declare_parameter("input_topic", "/camera/depth_points")
        self.declare_parameter("output_topic", "/depth_anything/points_filtered")
        self.declare_parameter("min_z", 0.15)
        self.declare_parameter("max_z", 0.80)
        self.declare_parameter("output_frame", "camera_link")
        self.declare_parameter("blind_spot", 0.5)
        self.declare_parameter("camera_height", 1.0)
        self.declare_parameter("camera_pitch_deg", 5.0)
        self.declare_parameter("log_interval", 30)

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self._min_z = self.get_parameter("min_z").get_parameter_value().double_value
        self._max_z = self.get_parameter("max_z").get_parameter_value().double_value
        self._output_frame = self.get_parameter("output_frame").get_parameter_value().string_value
        self._blind_spot = self.get_parameter("blind_spot").get_parameter_value().double_value
        self._camera_height = self.get_parameter("camera_height").get_parameter_value().double_value
        _pitch_deg = self.get_parameter("camera_pitch_deg").get_parameter_value().double_value
        self._log_interval = self.get_parameter("log_interval").get_parameter_value().integer_value

        # precompute pitch rotation trig (rotation around Y axis, right-hand rule)
        import math
        _pitch_rad = math.radians(_pitch_deg)
        self._cos_pitch = math.cos(_pitch_rad)
        self._sin_pitch = math.sin(_pitch_rad)

        # ---- static TF: base_link → camera_link (missing in current system) ----
        self._publish_static_tf()

        # ---- pub/sub ----
        self._sub = self.create_subscription(
            PointCloud2, input_topic, self._cloud_callback, 10
        )
        self._pub = self.create_publisher(PointCloud2, output_topic, 10)

        self._frame_seq = 0

        self.get_logger().info(
            f"GroundFilter ready | {input_topic} → {output_topic} | "
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
    #  PointCloud callback — 3D transform + spatial crop
    # ------------------------------------------------------------------

    def _cloud_callback(self, msg: PointCloud2):
        n_points = msg.width * msg.height if msg.height > 1 else msg.width
        if n_points == 0:
            return

        # ---- parse raw buffer → (N, 4) float32 [X_opt, Y_opt, Z_opt, intensity] ----
        data = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)
        X_opt = data[:, 0]
        Y_opt = data[:, 1]
        Z_opt = data[:, 2]
        intensity = data[:, 3]

        # ---- step 1: optical frame → camera standard frame ----
        #   optical:  Z=forward, Y=down,  X=right
        #   camera:   X=forward, Y=left,  Z=up
        X_cam = Z_opt
        Y_cam = -X_opt
        Z_cam = -Y_opt

        # ---- step 2: camera frame → base_link (pitch rotation around Y + height) ----
        #   R_y(pitch) * [X_cam, Y_cam, Z_cam]^T  then  Z += h
        cp = self._cos_pitch
        sp = self._sin_pitch
        X_base = X_cam * cp + Z_cam * sp
        Y_base = Y_cam
        Z_base = -X_cam * sp + Z_cam * cp + self._camera_height

        # ---- step 3: spatial crop in base_link frame ----
        #   X > blind_spot  (forward clearance)
        #   min_z < Z < max_z  (floating obstacle band)
        mask = (X_base > self._blind_spot) & (Z_base >= self._min_z) & (Z_base <= self._max_z)

        # ---- repack surviving points → (M, 4) ----
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
