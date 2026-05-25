"""depth_to_scan_node.py — Depth Anything V2 monocular depth -> pseudo-LaserScan bridge.

Architecture:
  MP4 file -> OpenCV -> DepthAnythingV2 (native, vits) -> /camera/depth_image (32FC1)
                                                           /camera/camera_info (fake intrinsics, latched)

ROS 2 iron laws enforced:
  1. Single timestamp per frame — Image and CameraInfo share the identical stamp.
  2. Unified frame_id — both use 'camera_link'.
  3. True 32FC1 metric depth — relative output scaled to [0.5, 10.0] metres.
  4. Valid intrinsics — fx=fy=500, cx=w/2, cy=h/2; dimensions from actual video frame.

Dependencies: depth_anything_v2 (local), torch, opencv, cv_bridge.
Zero transformers / HuggingFace.
"""

import os
import sys
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


MODEL_CONFIGS = {
    "vits": {"encoder": "vits", "features": 64,  "out_channels": [48,  96,  192, 384]},
    "vitb": {"encoder": "vitb", "features": 128, "out_channels": [96,  192, 384, 768]},
    "vitl": {"encoder": "vitl", "features": 256, "out_channels": [256, 512, 1024, 1024]},
}

FRAME_ID = "camera_link"


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


def _make_camera_info(width: int, height: int, stamp) -> CameraInfo:
    """Construct CameraInfo with valid phone-grade intrinsics.

    fx=fy=500, principal point at image centre, zero distortion.
    """
    fx = 500.0
    fy = 500.0
    cx = width / 2.0
    cy = height / 2.0

    msg = CameraInfo()
    msg.header.stamp = stamp
    msg.header.frame_id = FRAME_ID
    msg.height = height
    msg.width = width
    msg.distortion_model = "plumb_bob"
    msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    msg.k = [fx, 0.0, cx,
             0.0, fy, cy,
             0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0,
             0.0, fy, cy, 0.0,
             0.0, 0.0, 1.0, 0.0]
    return msg


class DepthToScanNode(Node):

    def __init__(self):
        super().__init__("depth_to_scan_node")

        # ---- parameters ----
        self.declare_parameter("video_path", "")
        self.declare_parameter("encoder", "vits")
        self.declare_parameter("weights_path", "")
        self.declare_parameter("depth_anything_home", "")
        self.declare_parameter("device", "auto")
        self.declare_parameter("inference_rate", 2.0)
        self.declare_parameter("depth_topic", "/camera/depth_image")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")

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
        depth_topic = (
            self.get_parameter("depth_topic").get_parameter_value().string_value
        )
        info_topic = (
            self.get_parameter("camera_info_topic").get_parameter_value().string_value
        )

        data_dir = _find_data_dir(__file__)

        # ---- resolve video ----
        if not video_path:
            video_path = os.path.join(data_dir, "test_video.mp4")

        # ---- resolve weights ----
        if not weights_path:
            weights_path = os.path.join(
                data_dir, f"depth_anything_v2_{encoder}.pth"
            )

        # ---- resolve depth_anything_v2 module path ----
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
            self.get_logger().fatal(
                f"Unknown encoder '{encoder}'. Choose: {list(MODEL_CONFIGS.keys())}"
            )
            raise ValueError(encoder)

        # ---- load model ----
        self._model = self._load_model(encoder, weights_path, device)

        # ---- video capture ----
        if not os.path.isfile(video_path):
            self.get_logger().fatal(f"Video not found: {video_path}")
            raise FileNotFoundError(video_path)
        self._cap = cv2.VideoCapture(video_path)
        if not self._cap.isOpened():
            self.get_logger().fatal(f"Cannot open video: {video_path}")
            raise RuntimeError(video_path)

        # ---- actual frame dimensions from video ----
        self._frame_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self._frame_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(
            f"Video: {video_path} | {self._frame_w}x{self._frame_h}"
        )

        # ---- cv_bridge ----
        self._bridge = CvBridge()

        # ---- publishers ----
        self._depth_pub = self.create_publisher(Image, depth_topic, 10)

        latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._info_pub = self.create_publisher(CameraInfo, info_topic, latched)

        # ---- timer ----
        self.create_timer(1.0 / rate, self._tick)

        self._frame_seq = 0
        self.get_logger().info(
            f"DepthToScan ready | encoder={encoder} | device={device} | "
            f"rate={rate} Hz | depth={depth_topic} | info={info_topic} | "
            f"frame_id={FRAME_ID}"
        )

    # ------------------------------------------------------------------
    #  Model
    # ------------------------------------------------------------------

    def _load_model(self, encoder: str, weights_path: str, device: str):
        import torch
        from depth_anything_v2.dpt import DepthAnythingV2

        if not os.path.isfile(weights_path):
            self.get_logger().fatal(f"Weights not found: {weights_path}")
            raise FileNotFoundError(weights_path)

        self.get_logger().info(
            f"Loading DepthAnythingV2 [{encoder}] from {weights_path} ..."
        )
        model = DepthAnythingV2(**MODEL_CONFIGS[encoder])
        state = torch.load(weights_path, map_location="cpu", weights_only=True)
        model.load_state_dict(state)
        model = model.to(device).eval()
        self.get_logger().info("Model loaded")
        return model

    # ------------------------------------------------------------------
    #  Tick — Iron Law 1: single timestamp for the entire frame
    # ------------------------------------------------------------------

    def _tick(self):
        ret, frame = self._cap.read()
        if not ret:
            self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = self._cap.read()
            if not ret:
                self.get_logger().warn("Cannot read frame, skipping")
                return

        # ---- ONE timestamp for both Image and CameraInfo ----
        stamp = self.get_clock().now().to_msg()

        depth = self._infer(frame)

        self._publish_depth(depth, stamp)
        self._publish_camera_info(stamp)

        self._frame_seq += 1
        self.get_logger().debug(
            f"Frame #{self._frame_seq} | {depth.shape[1]}x{depth.shape[0]} | "
            f"range=[{depth.min():.2f}, {depth.max():.2f}]m"
        )

    # ------------------------------------------------------------------
    #  Inference — Iron Law 3: metric depth [0.5, 10.0] m
    # ------------------------------------------------------------------

    def _infer(self, frame: np.ndarray) -> np.ndarray:
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        depth = self._model.infer_image(rgb)  # (H, W) float32, relative

        # Remap relative depth → metric [0.5, 10.0] metres.
        # DA2 outputs depth (higher = farther); if your model outputs
        # inverse depth (disparity, higher = closer), swap the comment below.
        d_min, d_max = depth.min(), depth.max()
        if d_max - d_min > 1e-6:
            depth = (depth - d_min) / (d_max - d_min)       # [0, 1]
            # depth = 1.0 - depth                           # uncomment if disparity
            depth = depth * 9.5 + 0.5                       # [0.5, 10.0]

        return depth.astype(np.float32)

    # ------------------------------------------------------------------
    #  Publishers
    # ------------------------------------------------------------------

    def _publish_depth(self, depth: np.ndarray, stamp):
        msg = self._bridge.cv2_to_imgmsg(depth, encoding="32FC1")
        msg.header.stamp = stamp
        msg.header.frame_id = FRAME_ID
        self._depth_pub.publish(msg)

    def _publish_camera_info(self, stamp):
        msg = _make_camera_info(self._frame_w, self._frame_h, stamp)
        self._info_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthToScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
