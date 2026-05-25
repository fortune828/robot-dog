"""sanitation_interfaces — 纯 Python 消息类型回退定义。

在无 ROS 2 编译环境时，节点可直接 import 这些 Python 数据类。
一旦 colcon build 完成 .msg 编译，节点应优先使用 ROS 2 原生消息类型。
"""

from dataclasses import dataclass, field
from typing import List


@dataclass
class DetectionResult:
    """单个检测目标"""
    class_name: str = ""
    confidence: float = 0.0
    bbox: List[int] = field(default_factory=lambda: [0, 0, 0, 0])  # [x, y, w, h]


@dataclass
class DetectionResultArray:
    """一帧内的全部检测结果"""
    detections: List[DetectionResult] = field(default_factory=list)

    def __len__(self):
        return len(self.detections)

    def __iter__(self):
        return iter(self.detections)

    def __getitem__(self, idx):
        return self.detections[idx]


__all__ = ["DetectionResult", "DetectionResultArray"]
