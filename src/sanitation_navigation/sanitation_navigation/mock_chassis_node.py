"""mock_chassis_node.py — 模拟底盘节点。

- 订阅 /cmd_vel (geometry_msgs/Twist) 速度指令
- 在终端日志中打印收到的控制量
- 模拟底盘运动学，发布 /odom (nav_msgs/Odometry) 里程计
- 发布 /tf base_link → odom 变换
"""

import math
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

try:
    from tf2_ros import TransformBroadcaster
    from geometry_msgs.msg import TransformStamped
    HAS_TF = True
except ImportError:
    HAS_TF = False


class MockChassisNode(Node):
    """模拟底盘 ROS 2 节点"""

    def __init__(self):
        super().__init__("mock_chassis_node")

        # ---- 参数 ----
        # odom (0, 0) 与 mock_gps_node 第一个路点对齐:
        #   path_latitudes[0] = 30.744154, path_longitudes[0] = 103.925233
        # 即底盘里程计原点对应成电图书馆 SW 角
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("update_rate", 20.0)  # Hz
        self.declare_parameter("gps_origin_lat", 30.744154)
        self.declare_parameter("gps_origin_lon", 103.925233)

        # ---- 订阅 ----
        cmd_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        self.subscription = self.create_subscription(
            Twist, cmd_topic, self._cmd_vel_callback, 10
        )

        # ---- 发布 ----
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)

        # ---- TF ----
        if HAS_TF:
            self._tf_broadcaster = TransformBroadcaster(self)
        else:
            self._tf_broadcaster = None

        # ---- 运动学状态 ----
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._linear_vel = 0.0
        self._angular_vel = 0.0
        self._last_time = self.get_clock().now()

        # ---- 定时器 ----
        rate = self.get_parameter("update_rate").get_parameter_value().double_value
        if rate <= 0.0:
            raise ValueError("update_rate must be positive")
        self._timer = self.create_timer(1.0 / rate, self._publish_odometry)

        origin_lat = self.get_parameter("gps_origin_lat").get_parameter_value().double_value
        origin_lon = self.get_parameter("gps_origin_lon").get_parameter_value().double_value
        self.get_logger().info(
            f"模拟底盘已启动 | odom(0,0) ↔ GPS({origin_lat:.4f}, {origin_lon:.4f}) | "
            f"等待 /cmd_vel 指令..."
        )

    def _cmd_vel_callback(self, msg: Twist):
        self._linear_vel = msg.linear.x
        self._angular_vel = msg.angular.z
        self.get_logger().debug(
            f"收到速度指令 | v={msg.linear.x:+.2f} m/s  "
            f"ω={msg.angular.z:+.2f} rad/s  "
            f"| 当前位置 ({self._x:.2f}, {self._y:.2f}, θ={math.degrees(self._theta):.1f}°)"
        )

    def _publish_odometry(self):
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self._last_time = now

        # 简单差速运动学
        delta_s = self._linear_vel * dt
        delta_theta = self._angular_vel * dt

        self._x += delta_s * math.cos(self._theta + delta_theta / 2.0)
        self._y += delta_s * math.sin(self._theta + delta_theta / 2.0)
        self._theta += delta_theta
        self._theta = math.atan2(math.sin(self._theta), math.cos(self._theta))

        # 构造 Odometry 消息
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.get_parameter("odom_frame").get_parameter_value().string_value
        odom.child_frame_id = self.get_parameter("base_frame").get_parameter_value().string_value

        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(self._theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self._theta / 2.0)

        odom.twist.twist.linear.x = self._linear_vel
        odom.twist.twist.angular.z = self._angular_vel

        # 协方差（Mock 设为极小值表示"确信"）
        for i in range(36):
            odom.pose.covariance[i] = 0.0
            odom.twist.covariance[i] = 0.0
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 0.01
        odom.pose.covariance[21] = 0.01
        odom.pose.covariance[28] = 0.01
        odom.pose.covariance[35] = 0.01

        self._odom_pub.publish(odom)

        # 发布 odom → base_link 变换
        if self._tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = odom.header.frame_id
            t.child_frame_id = odom.child_frame_id
            t.transform.translation.x = self._x
            t.transform.translation.y = self._y
            t.transform.translation.z = 0.0
            t.transform.rotation = odom.pose.pose.orientation
            self._tf_broadcaster.sendTransform(t)

            # 发布 base_link → gps_link 静态恒等变换
            t2 = TransformStamped()
            t2.header.stamp = now.to_msg()
            t2.header.frame_id = odom.child_frame_id
            t2.child_frame_id = "gps_link"
            t2.transform.translation.x = 0.0
            t2.transform.translation.y = 0.0
            t2.transform.translation.z = 0.0
            t2.transform.rotation.w = 1.0
            self._tf_broadcaster.sendTransform(t2)


def main(args=None):
    rclpy.init(args=args)
    node = MockChassisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
