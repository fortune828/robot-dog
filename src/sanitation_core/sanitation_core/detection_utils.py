"""检测相关的纯 Python 工具函数。

Mock YOLO 逻辑：在不依赖真实模型的情况下生成模拟检测结果，用于闭环测试。
生产环境下可替换为 ultralytics YOLO 调用。
"""

import random
from typing import List, Tuple

import numpy as np

# 模拟检测类别
MOCK_CLASSES = ["trash", "obstacle", "person"]


def generate_mock_detections(
    frame_shape: Tuple[int, int, int],
    detection_prob: float = 0.35,
    max_detections: int = 3,
    rng: random.Random | None = None,
) -> List[dict]:
    """在给定图像尺寸范围内生成随机的模拟检测结果。

    Args:
        frame_shape: (height, width, channels)
        detection_prob: 每一轮产生至少一个检测的概率
        max_detections: 单帧最多产生几个检测框
        rng: 可选，传入确定种子的 Random 实例以复现结果

    Returns:
        [{"class_name": str, "confidence": float, "bbox": [x, y, w, h]}, ...]
    """
    rng = rng or random.Random()
    h, w = frame_shape[0], frame_shape[1]

    if rng.random() > detection_prob:
        return []

    detections = []
    num = rng.randint(1, max_detections)
    for _ in range(num):
        cls = rng.choice(MOCK_CLASSES)
        conf = round(rng.uniform(0.45, 0.99), 2)
        bw = rng.randint(20, w // 3)
        bh = rng.randint(20, h // 3)
        bx = rng.randint(0, w - bw)
        by = rng.randint(0, h - bh)
        detections.append(
            {
                "class_name": cls,
                "confidence": conf,
                "bbox": [bx, by, bw, bh],
            }
        )
    return detections


def draw_detections(
    image: np.ndarray,
    detections: List[dict],
    color: Tuple[int, int, int] = (0, 255, 0),
    thickness: int = 2,
) -> np.ndarray:
    """在图像上绘制检测框和标签，返回新数组（不修改原图）。

    Args:
        image: BGR 格式的 numpy 图像 (H, W, 3)
        detections: generate_mock_detections 的输出格式
        color: 框的颜色 (B, G, R)
        thickness: 框线宽

    Returns:
        带标注的 BGR 图像副本
    """
    annotated = image.copy()
    for det in detections:
        x, y, w, h = det["bbox"]
        cls = det["class_name"]
        conf = det["confidence"]
        # 框
        cv2_draw_box(annotated, x, y, w, h, color, thickness)
        # 标签
        label = f"{cls} {conf:.2f}"
        cv2_draw_label(annotated, label, x, y - 8, color)
    return annotated


def cv2_draw_box(
    image: np.ndarray,
    x: int,
    y: int,
    w: int,
    h: int,
    color: Tuple[int, int, int],
    thickness: int,
) -> None:
    """用 numpy 切片绘制矩形框（无 cv2 依赖时的后备方案）"""
    h_img, w_img = image.shape[:2]
    x1, y1 = max(0, x), max(0, y)
    x2, y2 = min(w_img, x + w), min(h_img, y + h)
    # 上边
    if 0 <= y1 < h_img:
        image[y1, x1:x2] = color
    # 下边
    if 0 <= y2 - 1 < h_img:
        image[y2 - 1, x1:x2] = color
    # 左边
    if 0 <= x1 < w_img:
        image[y1:y2, x1] = color
    # 右边
    if 0 <= x2 - 1 < w_img:
        image[y1:y2, x2 - 1] = color


def cv2_draw_label(
    image: np.ndarray,
    text: str,
    x: int,
    y: int,
    color: Tuple[int, int, int],
) -> None:
    """简单的文本绘制后备方案（使用 numpy 绘制基本像素文字）。

    注意：这是一个极简实现，仅用于无 OpenCV 环境的 Mock 测试。
    """
    # 绘制底色条带
    char_w, char_h = 8, 12
    bar_w = len(text) * char_w
    bar_h = char_h + 4
    bx = max(0, x)
    by = max(0, y)
    h_img, w_img = image.shape[:2]
    bx2 = min(w_img, bx + bar_w)
    by2 = min(h_img, by + bar_h)
    # 半透明黑底
    bg = image[by:by2, bx:bx2].astype(np.float32)
    bg[:] = bg * 0.5 + np.array(color, dtype=np.float32) * 0.1
    image[by:by2, bx:bx2] = bg.astype(np.uint8)
