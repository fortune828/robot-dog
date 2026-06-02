"""depth_to_cloud_node.py — Depth Anything V2 → 3D PointCloud2.

Architecture:
  MP4 -> OpenCV -> DepthAnythingV2 -> pinhole back-projection (numpy, vectorised)
                                      -> /camera/depth_points (PointCloud2, camera_link)

No depthimage_to_laserscan. No CameraInfo. No 2D depth image.
Stride-based spatial downsampling keeps CPU usage low without a GPU kernel.
"""

import os
import sys
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField


MODEL_CONFIGS = {
    "vits": {"encoder": "vits", "features": 64,  "out_channels": [48,  96,  192, 384]},
    "vitb": {"encoder": "vitb", "features": 128, "out_channels": [96,  192, 384, 768]},
    "vitl": {"encoder": "vitl", "features": 256, "out_channels": [256, 512, 1024, 1024]},
}

FRAME_ID = "camera_link"

# ---- virtual intrinsics (pinhole, 640×480 target) ----
FX = 500.0
FY = 500.0
CX = 320.0
CY = 240.0


def _find_data_dir(start_path: str) -> str:
    p = Path(start_path)
    for parent in [p] + list(p.parents):
        candidate = parent / "data"
        if candidate.is_dir():
            return str(candidate)
    return str(Path.cwd() / "data")


def _find_depth_anything_home(data_dir: str) -> str:
    for root in [os.path.join(data_dir, "Depth-Anything-V2"), data_dir]:
        if os.path.isfile(os.path.join(root, "depth_anything_v2", "dpt.py")):
            return root
    return ""


class DepthToCloudNode(Node):
    """Depth Anything V2 → 3D point cloud via pinhole back-projection."""

    def __init__(self):
        super().__init__("depth_to_cloud_node")

        # ---- parameters ----
        self.declare_parameter("video_path", "")
        self.declare_parameter("encoder", "vits")
        self.declare_parameter("weights_path", "")
        self.declare_parameter("depth_anything_home", "")
        self.declare_parameter("device", "auto")
        self.declare_parameter("inference_rate", 10.0)
        self.declare_parameter("pointcloud_topic", "/camera/depth_points")
        self.declare_parameter("stride", 4)
        self.declare_parameter("min_depth", 0.1)
        self.declare_parameter("max_depth", 10.0)
        self.declare_parameter("camera_height", 0.0)   # 与 YAML 一致: 纯相机帧
        self.declare_parameter("target_width", 640)
        self.declare_parameter("target_height", 480)
        self.declare_parameter("depth_scale", 1.5)       # 与 YAML 一致
        self.declare_parameter("depth_inverse", False)   # True=逆深度(视差), False=正深度

        video_path = (
            self.get_parameter("video_path").get_parameter_value().string_value
        )
        encoder = (
            self.get_parameter("encoder").get_parameter_value().string_value
        )
        weights_path = (
            self.get_parameter("weights_path").get_parameter_value().string_value
        )
        da_home = (
            self.get_parameter("depth_anything_home").get_parameter_value().string_value
        )
        device = (
            self.get_parameter("device").get_parameter_value().string_value
        )
        rate = (
            self.get_parameter("inference_rate").get_parameter_value().double_value
        )
        pc_topic = (
            self.get_parameter("pointcloud_topic").get_parameter_value().string_value
        )
        self._stride = (
            self.get_parameter("stride").get_parameter_value().integer_value
        )
        self._min_depth = (
            self.get_parameter("min_depth").get_parameter_value().double_value
        )
        self._max_depth = (
            self.get_parameter("max_depth").get_parameter_value().double_value
        )
        self._camera_height = (
            self.get_parameter("camera_height").get_parameter_value().double_value
        )
        self._target_w = (
            self.get_parameter("target_width").get_parameter_value().integer_value
        )
        self._target_h = (
            self.get_parameter("target_height").get_parameter_value().integer_value
        )
        self._depth_scale = (
            self.get_parameter("depth_scale").get_parameter_value().double_value
        )
        self._depth_inverse = (
            self.get_parameter("depth_inverse").get_parameter_value().bool_value
        )

        data_dir = _find_data_dir(__file__)

        # ---- resolve paths ----
        if not video_path:
            video_path = os.path.join(data_dir, "test_video.mp4")
        if not weights_path:
            weights_path = os.path.join(data_dir, f"depth_anything_v2_{encoder}.pth")

        # ---- sys.path for local depth_anything_v2 ----
        if not da_home:
            da_home = _find_depth_anything_home(data_dir)
        if da_home and da_home not in sys.path:
            sys.path.insert(0, da_home)
            self.get_logger().info(f"sys.path += {da_home}")

        # ---- device ----
        if device == "auto":
            import torch
            device = "cuda" if torch.cuda.is_available() else "cpu"
        self._device = device

        if encoder not in MODEL_CONFIGS:
            self.get_logger().fatal(f"Unknown encoder '{encoder}'")
            raise ValueError(encoder)

        # ---- load model ----
        self._model = self._load_model(encoder, weights_path, device)

        # ---- video ----
        if not os.path.isfile(video_path):
            self.get_logger().fatal(f"Video not found: {video_path}")
            raise FileNotFoundError(video_path)
        self._cap = cv2.VideoCapture(video_path)
        if not self._cap.isOpened():
            self.get_logger().fatal(f"Cannot open video: {video_path}")
            raise RuntimeError(video_path)
        self.get_logger().info(
            f"Video: {video_path} | "
            f"native={int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
            f"{int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} | "
            f"target={self._target_w}x{self._target_h}"
        )

        # ---- precompute pixel grid (ROS: X-fwd, Y-left, Z-up) ----
        uu, vv = np.meshgrid(
            np.arange(0, self._target_w, self._stride, dtype=np.float32),
            np.arange(0, self._target_h, self._stride, dtype=np.float32),
        )
        self._py = -(uu - CX) / FX  # horizontal → Y (left +)
        self._pz = -(vv - CY) / FY  # vertical   → Z (up   +)

        self._pc_pub = self.create_publisher(PointCloud2, pc_topic, 10)
        self.create_timer(1.0 / rate, self._tick)

        self._frame_seq = 0
        n_points = self._py.size
        self.get_logger().info(
            f"DepthToCloud ready | encoder={encoder} | device={device} | "
            f"stride={self._stride} → ~{n_points} pts | "
            f"depth=[{self._min_depth}, {self._max_depth}]m | "
            f"topic={pc_topic} | frame_id={FRAME_ID}"
        )

    # ------------------------------------------------------------------
    #  Model
    # ------------------------------------------------------------------

    def _load_model(self, encoder, weights_path, device):
        import torch
        from depth_anything_v2.dpt import DepthAnythingV2

        if not os.path.isfile(weights_path):
            self.get_logger().fatal(f"Weights not found: {weights_path}")
            raise FileNotFoundError(weights_path)

        self.get_logger().info(f"Loading DepthAnythingV2 [{encoder}] ...")
        model = DepthAnythingV2(**MODEL_CONFIGS[encoder])
        state = torch.load(weights_path, map_location="cpu", weights_only=True)
        model.load_state_dict(state)
        model = model.to(device).eval()
        self.get_logger().info("Model loaded")
        return model

    # ------------------------------------------------------------------
    #  Tick
    # ------------------------------------------------------------------

    def _tick(self):
        ret, frame = self._cap.read()
        if not ret:
            self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = self._cap.read()
            if not ret:
                self.get_logger().warn("Cannot read frame, skipping")
                return

        stamp = self.get_clock().now().to_msg()

        # resize to target resolution before inference (saves model work)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb = cv2.resize(rgb, (self._target_w, self._target_h))

        depth = self._infer(rgb)
        cloud_msg = self._depth_to_cloud(depth, stamp)
        self._pc_pub.publish(cloud_msg)

        self._frame_seq += 1
        self.get_logger().debug(
            f"Frame #{self._frame_seq} | "
            f"{cloud_msg.width}x{cloud_msg.height} pts | "
            f"depth=[{depth.min():.2f}, {depth.max():.2f}]m"
        )

    # ------------------------------------------------------------------
    #  Inference → metric depth [min_depth, max_depth]
    #
    #  使用 P5-P95 百分位归一化（替代每帧 min-max）：
    #    - 避免单帧场景缩放导致的距离尺度漂移
    #    - 稳定的场景深度 → 真实物理尺度的映射
    #    - depth_scale 参数可用于校准（默认 1.0）
    # ------------------------------------------------------------------

    def _infer(self, rgb: np.ndarray) -> np.ndarray:
        depth = self._model.infer_image(rgb)  # (H, W) float32, relative

        # 如果模型输出的是视差（inverse depth），取反
        if self._depth_inverse:
            depth = -depth

        # 百分位归一化：截断首尾 5% 的异常值
        d_flat = depth.ravel()
        p5 = np.percentile(d_flat, 5)
        p95 = np.percentile(d_flat, 95)

        if p95 - p5 > 1e-6:
            depth = (depth - p5) / (p95 - p5)                     # [0, 1]
            depth = np.clip(depth, 0.0, 1.0)
            depth = depth * (self._max_depth - self._min_depth) * self._depth_scale + self._min_depth

        return depth.astype(np.float32)

    # ------------------------------------------------------------------
    #  Pinhole back-projection → PointCloud2 (vectorised)
    # ------------------------------------------------------------------

    def _depth_to_cloud(self, depth: np.ndarray, stamp) -> PointCloud2:
        """Pinhole back-projection → camera optical frame (pure, no height offset).

        point_x = depth                     forward distance
        point_y = -(u - cx) * depth / fx    left +  (horizontal)
        point_z = -(v - cy) * depth / fy    up   +  (vertical, pure camera origin)

        NOTE: Z is relative to camera optical center. Camera height offset
              is handled by the TF base_link→camera_link transform, NOT here.

        Fields: x, y, z, intensity (FLOAT32 × 4, point_step=16).
        """
        Z = depth[:: self._stride, :: self._stride]  # (H', W')

        Xf = Z                                                # forward
        Yf = self._py * Z                                    # left
        Zf = self._pz * Z                                    # up (pure camera frame, no height offset)

        mask = (Z > self._min_depth) & (Z < self._max_depth)

        # (N, 4) float32: x, y, z, intensity
        data = np.empty((mask.sum(), 4), dtype=np.float32)
        data[:, 0] = Xf[mask]
        data[:, 1] = Yf[mask]
        data[:, 2] = Zf[mask]
        data[:, 3] = Z[mask]   # intensity = depth

        n = data.shape[0]

        msg = PointCloud2()
        msg.header = Header(stamp=stamp, frame_id=FRAME_ID)
        msg.height = 1
        msg.width = n
        msg.fields = [
            PointField(name="x",         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y",         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z",         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 16
        msg.row_step = 16 * n
        msg.is_bigendian = False
        msg.is_dense = True
        msg.data = data.tobytes()

        return msg


def main(args=None):
    rclpy.init(args=args)
    node = DepthToCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
