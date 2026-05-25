"""mock_gps_node.py — 模拟 RTK GPS 节点。

- 订阅 /global_plan (nav_msgs/Path)，沿路径点匀速移动并发布 GPS 坐标
- 未收到 Path 时在原点附近发布带噪声的静止 GPS 信号
- 以 5 Hz 发布 sensor_msgs/NavSatFix + world→odom TF（包含 Yaw）
"""

import math
import random

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

try:
    from tf2_ros import TransformBroadcaster
    from geometry_msgs.msg import TransformStamped, Quaternion
    HAS_TF = True
except ImportError:
    HAS_TF = False

EARTH_RADIUS = 6371000.0


def _local_to_latlon(x, y, origin_lat, origin_lon):
    """局部 ENU (x, y) → (lat, lon)（逆 Equirectangular，使用精确公式）。
    
    Args:
        x, y: 局部平面坐标 (米)
        origin_lat: 投影原点纬度 (度)
        origin_lon: 投影原点经度 (度)
    
    Returns:
        (lat, lon) - 地理坐标 (度)
    """
    # 将 origin_lat 转换为弧度，用于 cos 计算
    origin_lat_rad = math.radians(origin_lat)
    
    # 使用精确的 ENU 反投影公式：
    # delta_lat = y / 111194.9 (单位: 度)
    # delta_lon = x / (111194.9 * cos(origin_lat_rad)) (单位: 度)
    # 其中 111194.9 ≈ EARTH_RADIUS * π / 180
    
    meters_per_degree = 111194.9
    
    delta_lat = y / meters_per_degree
    delta_lon = x / (meters_per_degree * math.cos(origin_lat_rad))
    
    lat = origin_lat + delta_lat
    lon = origin_lon + delta_lon
    
    return lat, lon


def _latlon_to_local(lat, lon, origin_lat, origin_lon):
    """(lat, lon) → 局部 ENU (x, y)（使用精确公式，与 gaode_path_proxy 一致）。"""
    origin_lat_rad = math.radians(origin_lat)
    meters_per_degree = 111194.9
    
    x = (lon - origin_lon) * meters_per_degree * math.cos(origin_lat_rad)
    y = (lat - origin_lat) * meters_per_degree
    return x, y


class MockGpsNode(Node):
    """模拟 RTK GPS 节点 — 空闲时静止噪声，收到 Path 后沿路径移动"""

    def __init__(self):
        super().__init__("mock_gps_node")

        self.declare_parameter("topic", "/fix")
        self.declare_parameter("publish_rate", 5.0)
        self.declare_parameter("speed", 10.0)
        self.declare_parameter("frame_id", "gps_link")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("plan_topic", "/global_plan")
        self.declare_parameter("origin_lat", 30.744154)
        self.declare_parameter("origin_lon", 103.925233)
        self.declare_parameter("noise_stddev", 0.000001)  # ~0.1m

        topic = self.get_parameter("topic").get_parameter_value().string_value
        self._pub = self.create_publisher(NavSatFix, topic, 10)

        self._speed = self.get_parameter("speed").get_parameter_value().double_value
        rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self._world_frame = self.get_parameter("world_frame").get_parameter_value().string_value
        self._plan_topic = self.get_parameter("plan_topic").get_parameter_value().string_value
        self._origin_lat = self.get_parameter("origin_lat").get_parameter_value().double_value
        self._origin_lon = self.get_parameter("origin_lon").get_parameter_value().double_value
        self._noise_stddev = self.get_parameter("noise_stddev").get_parameter_value().double_value

        # ---- TF ----
        if HAS_TF:
            self._tf_broadcaster = TransformBroadcaster(self)
        else:
            self._tf_broadcaster = None
            self.get_logger().warn("tf2_ros 不可用，跳过 TF 发布")

        # ---- 路径状态 ----
        self._path_poses = []          # 当前正在追踪的 Path 位姿列表
        self._has_path = False         # 是否已收到过 Path
        self._path_done = False        # 是否已走完
        self._goal_reported = False    # 到达终点后仅上报一次 REACHED_GOAL
        self._idle_lat = self._origin_lat  # 静止时的锚点纬度
        self._idle_lon = self._origin_lon  # 静止时的锚点经度
        self._current_yaw = 0.0        # 当前狗头的偏航角（弧度）
        
        # ========== 点到点直线追踪状态变量 ==========
        self._target_idx = 0           # 当前目标路径点索引
        self._current_x = 0.0          # 狗的当前 X 坐标（局部平面）
        self._current_y = 0.0          # 狗的当前 Y 坐标（局部平面）
        self._log_counter = 0          # 限频日志计数器（每 N 帧打印一次）

        # ---- 订阅 ----
        self._plan_sub = self.create_subscription(
            Path, self._plan_topic, self._path_callback, 10
        )

        # ---- 任务状态发布 ----
        self._mission_status_pub = self.create_publisher(String, "/mission_status", 10)

        # ---- 定时器 ----
        self._timer = self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"GPS 模拟已启动 | 原点=({self._origin_lat:.6f},{self._origin_lon:.6f}) | "
            f"等待 /global_plan ... | 空闲噪声 σ={self._noise_stddev:.1e}°"
        )

    # ------------------------------------------------------------------
    #  订阅回调
    # ------------------------------------------------------------------

    def _path_callback(self, msg: Path):
        """接收全局路径 — 不瞬移，狗从当前位置走向路径起点"""
        if not msg.poses or len(msg.poses) < 2:
            self.get_logger().warn("收到空路径或点数不足，忽略")
            return

        self.get_logger().info(
            f"\033[32m[INFO] Target Path received! Points: {len(msg.poses)}. "
            f"Dog will walk from here to the path...\033[0m"
        )

        self._path_poses = list(msg.poses)
        self._path_done = False
        self._goal_reported = False  # 新路径到达，重置上报标记
        self._target_idx = 0  # 路径第 0 个点是狗要去的第一个目标
        self._log_counter = 0  # 重置限频日志计数器

        if not self._has_path:
            self._has_path = True
            self.get_logger().info(
                f"\033[32m[SUCCESS] Gaode Path Received! "
                f"{len(self._path_poses)} waypoints, "
                f"Walking from current position...\033[0m"
            )
        else:
            self.get_logger().info(
                f"路径已更新 | {len(self._path_poses)} 点"
            )

    # ------------------------------------------------------------------
    #  定时回调（统一的 Timer 入口）
    # ------------------------------------------------------------------

    def _tick(self):
        """统一的 Timer 回调：步进控制 + GPS 发布 + TF 广播"""
        dt = self._timer.timer_period_ns * 1e-9  # 秒
        step = self._speed * dt                   # 单帧步进距离（10.0 m/s）

        # ========== 状态机 ==========
        if not self._path_poses or self._path_done:
            # 空闲：原点附近带噪声静止
            lat, lon, alt = self._noisy_idle()
        else:
            # 有路径：执行步进
            lat, lon, alt = self._step_along_path(step)

        # ========== 发布 GPS 定位 ==========
        self._publish_fix(lat, lon, alt)

        # ========== 公布 TF（无论空闲还是移动，必须广播） ==========
        self._publish_tf(self._current_x, self._current_y, self._current_yaw)

        # ========== 限频日志（每 5 秒一次） ==========
        if self._path_poses and not self._path_done:
            self._log_counter += 1
            if self._log_counter % 25 == 0:  # 5 Hz * 5 秒
                self.get_logger().info(
                    f"[MOVING] Moving to wp[{self._target_idx}], "
                    f"current_pos=({self._current_x:.2f}, {self._current_y:.2f})"
                )

    # ------------------------------------------------------------------
    #  静止噪声
    # ------------------------------------------------------------------

    def _noisy_idle(self):
        """空闲时在当前位置添加微小噪声，不改变内部状态"""
        lat = self._idle_lat + random.gauss(0.0, self._noise_stddev)
        lon = self._idle_lon + random.gauss(0.0, self._noise_stddev)
        return lat, lon, 0.0

    # ------------------------------------------------------------------
    #  沿 Path 行进（点到点直线追踪）
    # ------------------------------------------------------------------

    def _step_along_path(self, step):
        """点到点直线追踪 — 无瞬移、无递归、无死锁
        
        Args:
            step: 本帧步进距离 (米)
        Returns:
            (lat, lon, alt) — 当前 GPS 坐标
        """
        # 安全检查：目标索引超出范围 → 到达终点
        if self._target_idx >= len(self._path_poses):
            if not self._path_done:
                self._path_done = True
                self.get_logger().info(
                    f"\033[33m[ARRIVED] 已到达路径终点\033[0m"
                )
            # 保持当前位置不动
            lat, lon = _local_to_latlon(
                self._current_x, self._current_y,
                self._origin_lat, self._origin_lon
            )
            self._idle_lat = lat
            self._idle_lon = lon
            return lat, lon, 0.0

        # 当前目标点
        target = self._path_poses[self._target_idx].pose.position
        dx = target.x - self._current_x
        dy = target.y - self._current_y
        dist = math.hypot(dx, dy)

        # ========== 到达判定（0.2 米阈值） ==========
        if dist < 0.2:
            # 到达最后一个路点时上报任务完成
            if (not self._goal_reported
                    and self._target_idx == len(self._path_poses) - 1):
                self._goal_reported = True
                msg = String()
                msg.data = "REACHED_GOAL"
                self._mission_status_pub.publish(msg)
                self.get_logger().info(
                    "\033[33m[MISSION] REACHED_GOAL published\033[0m"
                )

            self.get_logger().info(
                f"\033[32m[INFO] Reached waypoint {self._target_idx}\033[0m"
            )
            self._target_idx += 1
            # 检查是否是最后一个点
            if self._target_idx >= len(self._path_poses):
                self._path_done = True
                self.get_logger().info(
                    f"\033[33m[ARRIVED] 已到达路径终点\033[0m"
                )
            # 本帧不再移动，返回当前位置
            lat, lon = _local_to_latlon(
                self._current_x, self._current_y,
                self._origin_lat, self._origin_lon
            )
            self._idle_lat = lat
            self._idle_lon = lon
            return lat, lon, 0.0

        # ========== 方向计算（稳健 atan2） ==========
        self._current_yaw = math.atan2(dy, dx)

        # ========== 匀速步进（按 yaw 方向而非 (dx/dist, dy/dist) 更稳定） ==========
        if dist <= step:
            # 一帧内能走完 → 走到目标点
            self._current_x = target.x
            self._current_y = target.y
        else:
            # 部分步进
            self._current_x += step * math.cos(self._current_yaw)
            self._current_y += step * math.sin(self._current_yaw)

        # ========== 转换为地理坐标 ==========
        lat, lon = _local_to_latlon(
            self._current_x, self._current_y,
            self._origin_lat, self._origin_lon
        )
        self._idle_lat = lat
        self._idle_lon = lon
        return lat, lon, 0.0

    # ------------------------------------------------------------------
    #  发布
    # ------------------------------------------------------------------

    def _publish_fix(self, lat, lon, alt):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps_link"
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = 0.0
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        msg.position_covariance[0] = 0.01
        msg.position_covariance[4] = 0.01
        msg.position_covariance[8] = 0.04
        msg.status.status = 0     # STATUS_FIX
        msg.status.service = 1    # GPS
        self._pub.publish(msg)

    def _publish_tf(self, x, y, yaw=0.0):
        """发布 world → odom 动态变换（局部平面坐标 + 方向）。"""
        if self._tf_broadcaster is None:
            return
        
        # 从 Yaw（绕 Z 轴旋转）计算四元数
        # 公式：当 roll=0, pitch=0 时
        # qx=0, qy=0, qz=sin(yaw/2), qw=cos(yaw/2)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self._world_frame
        tf_msg.child_frame_id = "odom"
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = 0.0
        # ========== 关键：设置旋转（狗头朝向） ==========
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockGpsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
