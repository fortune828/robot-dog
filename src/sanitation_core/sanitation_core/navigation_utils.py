"""导航与控制相关的纯 Python 工具函数。

提供纯追踪（Pure Pursuit）、路点管理等与 ROS 解耦的算法实现。
"""

import math
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class Pose2D:
    """二维位姿"""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0  # 弧度


@dataclass
class VelocityCmd:
    """速度指令"""
    linear: float = 0.0   # m/s
    angular: float = 0.0  # rad/s


@dataclass
class Waypoint:
    """路点"""
    x: float
    y: float
    label: str = ""


def normalize_angle(angle: float) -> float:
    """将角度归一化到 [-pi, pi)"""
    return math.atan2(math.sin(angle), math.cos(angle))


def compute_distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    """两点欧氏距离"""
    return math.hypot(b[0] - a[0], b[1] - a[1])


def pure_pursuit(
    current: Pose2D,
    target: Tuple[float, float],
    lookahead: float = 0.5,
    max_linear: float = 0.5,
    max_angular: float = 1.0,
    linear_gain: float = 0.8,
    angular_gain: float = 2.0,
) -> VelocityCmd:
    """纯追踪控制器。

    Args:
        current: 机器人当前位姿
        target: 目标路点 (x, y)
        lookahead: 前视距离 (m)
        max_linear: 最大线速度 (m/s)
        max_angular: 最大角速度 (rad/s)
        linear_gain: 线速度比例增益
        angular_gain: 角速度比例增益

    Returns:
        VelocityCmd 速度指令
    """
    dx = target[0] - current.x
    dy = target[1] - current.y
    dist = math.hypot(dx, dy)

    if dist < 0.05:
        return VelocityCmd(0.0, 0.0)

    target_angle = math.atan2(dy, dx)
    angle_error = normalize_angle(target_angle - current.theta)

    cmd = VelocityCmd()
    cmd.linear = min(max_linear, dist * linear_gain)
    cmd.angular = max(-max_angular, min(max_angular, angle_error * angular_gain))

    # 角度偏差过大时减速
    if abs(angle_error) > math.pi / 3:
        cmd.linear *= 0.3

    return cmd


def select_next_waypoint(
    waypoints: List[Waypoint],
    current_idx: int,
    current_pos: Tuple[float, float],
    arrival_radius: float = 0.15,
) -> int:
    """检查当前路点是否到达，返回下一个路点索引（循环）。

    Args:
        waypoints: 路点列表
        current_idx: 当前目标路点索引
        current_pos: 机器人当前位置 (x, y)
        arrival_radius: 到达判定半径 (m)

    Returns:
        下一个应追踪的路点索引
    """
    if not waypoints:
        return 0

    target = waypoints[current_idx]
    if compute_distance(current_pos, (target.x, target.y)) < arrival_radius:
        return (current_idx + 1) % len(waypoints)
    return current_idx


def build_default_waypoints() -> List[Waypoint]:
    """构造默认的方形巡逻路点列表（4 个角点）。"""
    return [
        Waypoint(1.0, 0.0, "A"),
        Waypoint(1.0, 1.0, "B"),
        Waypoint(0.0, 1.0, "C"),
        Waypoint(0.0, 0.0, "D"),
    ]
