"""测试 sanitation_core.detection_utils — 检测纯函数"""

import random
import numpy as np
import pytest

from sanitation_core.detection_utils import (
    generate_mock_detections,
    draw_detections,
    MOCK_CLASSES,
)


class TestGenerateMockDetections:
    """generate_mock_detections 单元测试"""

    def test_empty_when_prob_zero(self):
        dets = generate_mock_detections((480, 640, 3), detection_prob=0.0)
        assert dets == []

    def test_always_produces_result_when_prob_one(self):
        dets = generate_mock_detections((480, 640, 3), detection_prob=1.0, max_detections=1)
        assert len(dets) >= 1

    def test_respects_max_detections(self):
        dets = generate_mock_detections((480, 640, 3), detection_prob=1.0, max_detections=5)
        assert len(dets) <= 5

    def test_seeded_reproducibility(self):
        rng1 = random.Random(42)
        rng2 = random.Random(42)
        d1 = generate_mock_detections((480, 640, 3), detection_prob=1.0, max_detections=3, rng=rng1)
        d2 = generate_mock_detections((480, 640, 3), detection_prob=1.0, max_detections=3, rng=rng2)
        assert d1 == d2

    def test_bbox_within_frame(self):
        h, w = 480, 640
        dets = generate_mock_detections((h, w, 3), detection_prob=1.0, max_detections=20, rng=random.Random(0))
        for d in dets:
            x, y, bw, bh = d["bbox"]
            assert 0 <= x < w
            assert 0 <= y < h
            assert x + bw <= w
            assert y + bh <= h

    def test_class_names_valid(self):
        dets = generate_mock_detections((480, 640, 3), detection_prob=1.0, max_detections=10)
        for d in dets:
            assert d["class_name"] in MOCK_CLASSES

    def test_confidence_in_range(self):
        dets = generate_mock_detections((480, 640, 3), detection_prob=1.0, max_detections=10)
        for d in dets:
            assert 0.45 <= d["confidence"] <= 0.99


class TestDrawDetections:
    """draw_detections 单元测试"""

    def test_returns_copy_not_same_object(self):
        img = np.zeros((100, 100, 3), dtype=np.uint8)
        dets = [{"class_name": "trash", "confidence": 0.9, "bbox": [10, 10, 30, 30]}]
        result = draw_detections(img, dets)
        assert result is not img
        assert result.shape == img.shape

    def test_empty_detections_returns_unchanged_copy(self):
        img = np.ones((100, 100, 3), dtype=np.uint8) * 128
        result = draw_detections(img, [])
        assert np.array_equal(result, img)

    def test_drawing_modifies_pixels(self):
        img = np.zeros((200, 200, 3), dtype=np.uint8)
        dets = [{"class_name": "obstacle", "confidence": 0.8, "bbox": [50, 50, 80, 80]}]
        result = draw_detections(img, dets)
        # 框区域应该被修改
        assert not np.array_equal(result[50:130, 50:130], img[50:130, 50:130])
