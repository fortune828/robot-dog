"""waypoint_patrol_node.py — 路点巡检节点。

- 读取预设路点列表（YAML 参数或默认方形路径）
- 订阅 /odom 获取当前位置
- 使用纯追踪控制器计算速度指令
- 发布 /cmd_vel (geometry_msgs/Twist)
"""

import math
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from sanitation_core.navigation_utils import (
    Pose2D,
    VelocityCmd,
    Waypoint,
    normalize_angle,
    pure_pursuit,
    select_next_waypoint,
    build_default_waypoints,
)


class WaypointPatrolNode(Node):
    """路点巡检 ROS 2 节点"""

    def __init__(self):
        super().__init__("waypoint_patrol_node")

        # ---- 参数 ----
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("control_rate", 10.0)  # Hz
        self.declare_parameter("arrival_radius", 0.15)  # m
        self.declare_parameter("lookahead", 0.5)  # m
        self.declare_parameter("max_linear", 0.4)  # m/s
        self.declare_parameter("max_angular", 0.8)  # rad/s
        # 路点: 一维平铺浮点数 [x0, y0, x1, y1, ...]
        self.declare_parameter(
            "waypoints",
            [1.0, 0.0, 1.0, 1.0, 0.0, 1.0, 0.0, 0.0],
        )

        # ---- 加载路点 ----
        raw_val = self.get_parameter("waypoints").value
        self._waypoints = self._parse_waypoints(raw_val)
        if not self._waypoints:
            self.get_logger().warn("路点参数为空，使用默认方形路径")
            self._waypoints = build_default_waypoints()

        self._current_wp_idx = 0
        self._pose = Pose2D()
        self._odom_received = False

        # ---- 发布 ----
        cmd_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        self._cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

        # ---- 订阅 ----
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self._odom_sub = self.create_subscription(
            Odometry, odom_topic, self._odom_callback, 10
        )

        # ---- 定时控制循环 ----
        rate = self.get_parameter("control_rate").get_parameter_value().double_value
        self._timer = self.create_timer(1.0 / rate, self._control_loop)

        self.get_logger().info(
            f"巡检节点已启动，加载 {len(self._waypoints)} 个路点: "
            f"{[f'{w.label}({w.x:.1f},{w.y:.1f})' for w in self._waypoints]}"
        )

    @staticmethod
    def _parse_waypoints(raw_val) -> list:
        """将一维平铺浮点数数组按步长 2 解析为 Waypoint 列表"""
        wps = []
        if isinstance(raw_val, (list, tuple)):
            arr = [float(v) for v in raw_val]
            for i in range(0, len(arr) - 1, 2):
                wps.append(Waypoint(arr[i], arr[i + 1], f"wp{i // 2}"))
        return wps

    def _odom_callback(self, msg: Odometry):
        self._pose.x = msg.pose.pose.position.x
        self._pose.y = msg.pose.pose.position.y
        # 从四元数提取 yaw
        q = msg.pose.pose.orientation
        self._pose.theta = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        if not self._odom_received:
            self.get_logger().info("已收到里程计数据，开始巡检控制")
            self._odom_received = True

    def _control_loop(self):
        if not self._waypoints:
            return

        target = self._waypoints[self._current_wp_idx]
        arrival = self.get_parameter("arrival_radius").get_parameter_value().double_value
        lookahead = self.get_parameter("lookahead").get_parameter_value().double_value
        max_lin = self.get_parameter("max_linear").get_parameter_value().double_value
        max_ang = self.get_parameter("max_angular").get_parameter_value().double_value

        # 检查是否到达
        new_idx = select_next_waypoint(
            self._waypoints, self._current_wp_idx,
            (self._pose.x, self._pose.y), arrival,
        )
        if new_idx != self._current_wp_idx:
            prev = self._waypoints[self._current_wp_idx]
            self._current_wp_idx = new_idx
            nxt = self._waypoints[new_idx]
            self.get_logger().info(
                f"到达路点 {prev.label}({prev.x:.1f},{prev.y:.1f}) "
                f"→ 切换至 {nxt.label}({nxt.x:.1f},{nxt.y:.1f})"
            )

        target = self._waypoints[self._current_wp_idx]
        cmd = pure_pursuit(
            self._pose,
            (target.x, target.y),
            lookahead=lookahead,
            max_linear=max_lin,
            max_angular=max_ang,
        )

        twist = Twist()
        twist.linear.x = cmd.linear
        twist.angular.z = cmd.angular
        self._cmd_pub.publish(twist)

        # 日志
        dx = target.x - self._pose.x
        dy = target.y - self._pose.y
        dist = math.hypot(dx, dy)
        self.get_logger().info(
            f"路点 {self._current_wp_idx} {target.label} | "
            f"距离={dist:.2f}m | "
            f"指令 v={cmd.linear:+.2f} ω={cmd.angular:+.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
