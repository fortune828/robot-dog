"""测试 sanitation_core.navigation_utils — 导航纯函数"""

import math
import pytest

from sanitation_core.navigation_utils import (
    Pose2D,
    VelocityCmd,
    Waypoint,
    normalize_angle,
    compute_distance,
    pure_pursuit,
    select_next_waypoint,
    build_default_waypoints,
)


class TestNormalizeAngle:
    def test_zero(self):
        assert normalize_angle(0.0) == pytest.approx(0.0)

    def test_pi_boundary(self):
        # atan2(0, -1) = +pi，与 -pi 等价，均合法
        result = normalize_angle(math.pi)
        assert result == pytest.approx(math.pi, abs=1e-6) or result == pytest.approx(-math.pi, abs=1e-6)

    def test_large_angle(self):
        result = normalize_angle(3 * math.pi)
        assert result == pytest.approx(math.pi, abs=1e-6) or result == pytest.approx(-math.pi, abs=1e-6)

    def test_negative_pi(self):
        assert normalize_angle(-math.pi) == pytest.approx(-math.pi, abs=1e-6)


class TestComputeDistance:
    def test_same_point(self):
        assert compute_distance((0, 0), (0, 0)) == 0.0

    def test_unit_distance(self):
        assert compute_distance((0, 0), (1, 0)) == 1.0

    def test_diagonal(self):
        assert compute_distance((0, 0), (3, 4)) == 5.0


class TestPurePursuit:
    def test_at_target_produces_zero_cmd(self):
        cmd = pure_pursuit(Pose2D(0, 0, 0), (0, 0))
        assert cmd.linear == 0.0
        assert cmd.angular == 0.0

    def test_straight_ahead(self):
        """目标正前方，角度误差为 0，只应有前向速度"""
        cmd = pure_pursuit(Pose2D(0, 0, 0), (2, 0), lookahead=0.5, max_linear=0.5)
        assert cmd.linear > 0.0
        assert cmd.angular == pytest.approx(0.0, abs=1e-6)

    def test_target_left_produces_positive_angular(self):
        """目标在左侧，应产生正角速度"""
        cmd = pure_pursuit(Pose2D(0, 0, 0), (0, 1))
        assert cmd.angular > 0.0

    def test_target_right_produces_negative_angular(self):
        """目标在右侧，应产生负角速度"""
        cmd = pure_pursuit(Pose2D(0, 0, 0), (0, -1))
        assert cmd.angular < 0.0

    def test_respects_max_limits(self):
        cmd = pure_pursuit(Pose2D(0, 0, 0), (100, 100), max_linear=0.3, max_angular=0.5)
        assert cmd.linear <= 0.3
        assert abs(cmd.angular) <= 0.5

    def test_lookahead_changes_turn_rate(self):
        pose = Pose2D(0, 0, 0)
        tight = pure_pursuit(pose, (1, 1), lookahead=0.25, max_angular=10.0)
        smooth = pure_pursuit(pose, (1, 1), lookahead=1.0, max_angular=10.0)
        assert abs(tight.angular) > abs(smooth.angular)

    def test_large_angle_error_reduces_speed(self):
        """大角度偏差时应减速"""
        # 目标在背后
        cmd_back = pure_pursuit(Pose2D(0, 0, math.pi / 2), (10, -10))
        assert cmd_back.linear < 0.3  # 应明显减速


class TestSelectNextWaypoint:
    def test_arrival_switches_index(self):
        wps = build_default_waypoints()
        # 机器人已在路点 0 的位置
        new_idx = select_next_waypoint(wps, 0, (1.0, 0.0), arrival_radius=0.2)
        assert new_idx == 1

    def test_not_arrived_keeps_index(self):
        wps = build_default_waypoints()
        # 机器人离路点 0 很远
        new_idx = select_next_waypoint(wps, 0, (0.0, 0.0), arrival_radius=0.2)
        assert new_idx == 0

    def test_wraps_around(self):
        wps = build_default_waypoints()
        # 最后一路点到达后回到 0
        new_idx = select_next_waypoint(wps, 3, (0.0, 0.0), arrival_radius=0.2)
        assert new_idx == 0

    def test_empty_waypoints(self):
        assert select_next_waypoint([], 0, (0, 0)) == 0


class TestBuildDefaultWaypoints:
    def test_returns_four_waypoints(self):
        wps = build_default_waypoints()
        assert len(wps) == 4

    def test_all_have_labels(self):
        wps = build_default_waypoints()
        for w in wps:
            assert w.label in ("A", "B", "C", "D")
