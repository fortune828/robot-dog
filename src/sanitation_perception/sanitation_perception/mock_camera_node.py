"""mock_camera_node.py — 模拟摄像头节点。

- 优先读取 data/videos/test_video.mp4 视频文件
- 若无视频文件，则生成纯色+时间戳的合成图像
- 以 15 Hz 频率发布 sensor_msgs/Image 话题
"""

import os
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

# 可选依赖：cv_bridge 和 cv2（无 ROS 2 环境时自动降级）
try:
    from cv_bridge import CvBridge
    HAS_CV_BRIDGE = True
except ImportError:
    HAS_CV_BRIDGE = False

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

import numpy as np

FRAME_H = 480
FRAME_W = 640
DEFAULT_RATE = 15.0  # Hz


def _find_data_dir(start_path: str) -> str:
    """向上查找项目根目录下的 data/ 目录"""
    p = Path(start_path)
    for parent in [p] + list(p.parents):
        candidate = parent / "data"
        if candidate.is_dir():
            return str(candidate)
    return str(Path.cwd() / "data")


class MockCameraNode(Node):
    """模拟摄像头 ROS 2 节点"""

    def __init__(self):
        super().__init__("mock_camera_node")

        # 声明参数
        self.declare_parameter("video_path", "")
        self.declare_parameter("frame_width", FRAME_W)
        self.declare_parameter("frame_height", FRAME_H)
        self.declare_parameter("publish_rate", DEFAULT_RATE)
        self.declare_parameter("topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("frame_id", "camera_optical_frame")
        self.declare_parameter("focal_length_px", 480.0)
        self.declare_parameter("principal_point_x", -1.0)
        self.declare_parameter("principal_point_y", -1.0)

        topic = self.get_parameter("topic").get_parameter_value().string_value
        camera_info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.publisher = self.create_publisher(Image, topic, 10)
        self.camera_info_publisher = self.create_publisher(CameraInfo, camera_info_topic, 10)

        rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / rate, self._publish_frame)

        self._bridge = CvBridge() if HAS_CV_BRIDGE else None
        self._frame_w = self.get_parameter("frame_width").get_parameter_value().integer_value
        self._frame_h = self.get_parameter("frame_height").get_parameter_value().integer_value
        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self._focal_length_px = self.get_parameter("focal_length_px").get_parameter_value().double_value
        self._principal_x = self.get_parameter("principal_point_x").get_parameter_value().double_value
        self._principal_y = self.get_parameter("principal_point_y").get_parameter_value().double_value
        if rate <= 0.0 or self._frame_w <= 0 or self._frame_h <= 0:
            raise ValueError("publish_rate and frame dimensions must be positive")
        if self._focal_length_px <= 0.0:
            raise ValueError("focal_length_px must be positive")
        self._cap = None
        self._frame_seq = 0

        # 尝试打开视频
        video_path = self.get_parameter("video_path").get_parameter_value().string_value
        if not video_path:
            video_path = os.path.join(_find_data_dir(__file__), "videos", "test_video.mp4")

        if HAS_CV2 and video_path and os.path.isfile(video_path):
            self._cap = cv2.VideoCapture(video_path)
            if self._cap.isOpened():
                self.get_logger().info(f"正在读取视频: {video_path}")
            else:
                self.get_logger().warn(f"无法打开视频文件 {video_path}，将使用合成图像")
                self._cap = None
        else:
            if not HAS_CV2:
                self.get_logger().warn("OpenCV 未安装，将使用 numpy 合成图像")
            else:
                self.get_logger().info(
                    f"视频文件 '{video_path}' 不存在，将使用 numpy 合成图像"
                )

    def _publish_frame(self):
        frame = self._read_frame()
        stamp = self.get_clock().now().to_msg()
        msg = self._frame_to_msg(frame, stamp)
        if msg is not None:
            self.publisher.publish(msg)
            self.camera_info_publisher.publish(self._make_camera_info(stamp))
            self.get_logger().debug(
                f"发布图像帧 #{self._frame_seq}  {self._frame_w}x{self._frame_h}"
            )
        self._frame_seq += 1

    def _read_frame(self) -> np.ndarray:
        """获取一帧：优先视频，其次合成"""
        if self._cap and self._cap.isOpened():
            ret, frame = self._cap.read()
            if ret:
                return cv2.resize(frame, (self._frame_w, self._frame_h))
            # 视频播完，循环
            self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = self._cap.read()
            if ret:
                return cv2.resize(frame, (self._frame_w, self._frame_h))

        return self._synthetic_frame()

    def _synthetic_frame(self) -> np.ndarray:
        """生成包含序列号和时戳的合成图像"""
        frame = np.zeros((self._frame_h, self._frame_w, 3), dtype=np.uint8)
        # 渐变背景
        for row in range(self._frame_h):
            intensity = int(60 + 40 * row / self._frame_h)
            frame[row, :] = [intensity, intensity // 2, 0]
        # 十字线
        cv_y, cv_x = self._frame_h // 2, self._frame_w // 2
        frame[cv_y - 2 : cv_y + 3, :] = [0, 255, 255]
        frame[:, cv_x - 2 : cv_x + 3] = [0, 255, 255]
        return frame

    def _frame_to_msg(self, frame: np.ndarray, stamp):
        """将 numpy 帧转为 ROS Image 消息"""
        if self._bridge is not None:
            msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = stamp
            msg.header.frame_id = self._frame_id
            return msg

        # 纯 numpy 后备方案：手动构造 Image 消息
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = frame.shape[2] * frame.shape[1]
        msg.data = frame.tobytes()
        return msg

    def _make_camera_info(self, stamp) -> CameraInfo:
        """Build synchronized approximate pinhole intrinsics for video experiments."""
        cx = self._principal_x if self._principal_x >= 0.0 else (self._frame_w - 1) / 2.0
        cy = self._principal_y if self._principal_y >= 0.0 else (self._frame_h - 1) / 2.0
        focal = self._focal_length_px
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.height = self._frame_h
        msg.width = self._frame_w
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [focal, 0.0, cx, 0.0, 0.0, focal, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = MockCameraNode()
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
