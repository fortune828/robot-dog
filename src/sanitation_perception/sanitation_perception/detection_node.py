"""detection_node.py — 目标检测节点。

- 订阅 /camera/image_raw (sensor_msgs/Image)
- 借助 sanitation_core 的 Mock YOLO 逻辑生成模拟检测结果
- 发布带框标注图像 /detection/annotated_image
- 发布检测结果 /detection/results (JSON 字符串)
"""

import json
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    from cv_bridge import CvBridge
    HAS_CV_BRIDGE = True
except ImportError:
    HAS_CV_BRIDGE = False

import numpy as np

from sanitation_core.detection_utils import generate_mock_detections, draw_detections


class DetectionNode(Node):
    """目标检测 ROS 2 节点（Mock 模式）"""

    def __init__(self):
        super().__init__("detection_node")

        # ---- 参数 ----
        self.declare_parameter("input_topic", "/camera/image_raw")
        self.declare_parameter("annotated_topic", "/detection/annotated_image")
        self.declare_parameter("results_topic", "/detection/results")
        self.declare_parameter("detection_prob", 0.4)
        self.declare_parameter("max_detections", 3)

        # ---- 订阅 ----
        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        self.subscription = self.create_subscription(
            Image, input_topic, self._image_callback, 10
        )

        # ---- 发布 ----
        annotated_topic = (
            self.get_parameter("annotated_topic").get_parameter_value().string_value
        )
        results_topic = (
            self.get_parameter("results_topic").get_parameter_value().string_value
        )
        self._annotated_pub = self.create_publisher(Image, annotated_topic, 10)
        self._results_pub = self.create_publisher(String, results_topic, 10)

        self._bridge = CvBridge() if HAS_CV_BRIDGE else None
        self.get_logger().info("检测节点已启动（Mock YOLO 模式）")

    def _image_callback(self, msg: Image):
        # 解码图像
        if self._bridge is not None:
            try:
                frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception as e:
                self.get_logger().error(f"图像解码失败: {e}")
                return
        else:
            # 纯 numpy 解码
            frame = self._decode_image_raw(msg)

        if frame is None:
            return

        # ---- 核心逻辑（纯函数调用）----
        det_prob = self.get_parameter("detection_prob").get_parameter_value().double_value
        max_det = self.get_parameter("max_detections").get_parameter_value().integer_value

        detections = generate_mock_detections(
            frame.shape, detection_prob=det_prob, max_detections=max_det
        )

        # 发布检测结果（JSON 字符串）
        result_msg = String()
        result_msg.data = json.dumps(detections, ensure_ascii=False)
        self._results_pub.publish(result_msg)

        # 绘制并发布标注图像
        annotated = draw_detections(frame, detections)
        if self._bridge is not None:
            out_msg = self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        else:
            out_msg = self._encode_image_raw(annotated, msg.header)
        out_msg.header.stamp = msg.header.stamp
        out_msg.header.frame_id = msg.header.frame_id
        self._annotated_pub.publish(out_msg)

        if detections:
            names = ", ".join(
                f"{d['class_name']}({d['confidence']:.2f})" for d in detections
            )
            self.get_logger().info(f"检测到 {len(detections)} 个目标: {names}")
        else:
            self.get_logger().debug("当前帧未检测到目标")

    @staticmethod
    def _decode_image_raw(msg: Image) -> np.ndarray | None:
        """无 cv_bridge 时的后备图像解码（仅支持 bgr8）"""
        if msg.encoding not in ("bgr8", "rgb8"):
            return None
        data = np.frombuffer(msg.data, dtype=np.uint8)
        try:
            frame = data.reshape((msg.height, msg.width, 3))
            return frame[:, :, ::-1] if msg.encoding == "rgb8" else frame
        except ValueError:
            return None

    @staticmethod
    def _encode_image_raw(frame: np.ndarray, header) -> Image:
        """无 cv_bridge 时的后备图像编码"""
        msg = Image()
        msg.header = header
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = frame.shape[2] * frame.shape[1]
        msg.data = frame.tobytes()
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
