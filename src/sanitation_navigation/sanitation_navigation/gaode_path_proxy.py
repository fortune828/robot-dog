"""gaode_path_proxy.py — 高德步行路径规划代理节点。

- 订阅 /fix (NavSatFix) 获取当前 WGS-84 位置
- 订阅 /navigation_goal (PoseStamped) 接收导航终点
- 调用高德步行路径规划 API 获取路径
- GCJ-02 ↔ WGS-84 坐标系双向转换
- 解码高德 polyline 并发布 nav_msgs/Path 到 /global_plan
"""

import json
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix

try:
    import requests
    HAS_REQUESTS = True
except ImportError:
    HAS_REQUESTS = False

AMAP_WALKING_URL = "https://restapi.amap.com/v3/direction/walking"
EARTH_RADIUS = 6371000.0
EE = 0.00669342162296594323  # WGS-84 第一偏心率平方
SEMI_MAJOR = 6378245.0       # GCJ-02 长半轴


# ============================================================================
#  GCJ-02 ↔ WGS-84 坐标转换
# ============================================================================

def _transform_lat(x, y):
    ret = (-100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y
           + 0.1 * x * y + 0.2 * math.sqrt(abs(x)))
    ret += (20.0 * math.sin(6.0 * x * math.pi)
            + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(y * math.pi)
            + 40.0 * math.sin(y / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (160.0 * math.sin(y / 12.0 * math.pi)
            + 320.0 * math.sin(y * math.pi / 30.0)) * 2.0 / 3.0
    return ret


def _transform_lng(x, y):
    ret = (300.0 + x + 2.0 * y + 0.1 * x * x
           + 0.1 * x * y + 0.1 * math.sqrt(abs(x)))
    ret += (20.0 * math.sin(6.0 * x * math.pi)
            + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(x * math.pi)
            + 40.0 * math.sin(x / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (150.0 * math.sin(x / 12.0 * math.pi)
            + 300.0 * math.sin(x / 30.0 * math.pi)) * 2.0 / 3.0
    return ret


def _delta(lat, lng):
    rad_lat = lat / 180.0 * math.pi
    magic = 1.0 - EE * math.sin(rad_lat) ** 2
    sqrt_magic = math.sqrt(magic)
    dlat = (_transform_lat(lng - 105.0, lat - 35.0) * 180.0
            / ((SEMI_MAJOR * (1.0 - EE)) / (magic * sqrt_magic) * math.pi))
    dlng = (_transform_lng(lng - 105.0, lat - 35.0) * 180.0
            / (SEMI_MAJOR / sqrt_magic * math.cos(rad_lat) * math.pi))
    return dlat, dlng


def wgs84_to_gcj02(lng, lat):
    """WGS-84 → GCJ-02（火星坐标系）。"""
    dlat, dlng = _delta(lat, lng)
    return lng + dlng, lat + dlat


def gcj02_to_wgs84(lng, lat):
    """GCJ-02 → WGS-84（迭代修正法）。"""
    dlat, dlng = _delta(lat, lng)
    wgs_lng = lng - dlng
    wgs_lat = lat - dlat
    # 一次迭代修正以提高精度
    dlat2, dlng2 = _delta(wgs_lat, wgs_lng)
    return lng - dlng2, lat - dlat2



# ============================================================================
#  局部平面投影
# ============================================================================

def _latlon_to_local(lat, lon, origin_lat, origin_lon, alt=0.0):
    """(lat, lon) → 局部 ENU (x, y, z)（使用精确公式）。
    
    Args:
        lat, lon: 地理坐标 (度)
        origin_lat: 投影原点纬度 (度)
        origin_lon: 投影原点经度 (度)
        alt: 高度 (米)
    
    Returns:
        (x, y, z) - 局部平面坐标 (米)
    """
    # 将 origin_lat 转换为弧度，用于 cos 计算
    origin_lat_rad = math.radians(origin_lat)
    
    # 使用精确的 ENU 投影公式：
    # x = (lon - origin_lon) * 111194.9 * cos(origin_lat_rad)
    # y = (lat - origin_lat) * 111194.9
    # 其中 111194.9 ≈ EARTH_RADIUS * π / 180
    
    meters_per_degree = 111194.9
    
    x = (lon - origin_lon) * meters_per_degree * math.cos(origin_lat_rad)
    y = (lat - origin_lat) * meters_per_degree
    z = alt
    
    return x, y, z


def _local_to_latlon(x, y, origin_lat, origin_lon):
    """局部 (x, y) → (lat, lon)（逆 Equirectangular）。"""
    cos_phi = math.cos(math.radians(origin_lat))
    lon = origin_lon + x / (EARTH_RADIUS * (math.pi / 180.0) * cos_phi)
    lat = origin_lat + y / (EARTH_RADIUS * (math.pi / 180.0))
    return lat, lon


# ============================================================================
#  ROS 2 节点
# ============================================================================

class GaodePathProxy(Node):
    """高德步行路径规划代理"""

    def __init__(self):
        super().__init__("gaode_path_proxy")

        # ---- 参数 ----
        self.declare_parameter("amap_api_key", "")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("gps_topic", "/fix")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("plan_topic", "/global_plan")

        self._api_key = self.get_parameter("amap_api_key").get_parameter_value().string_value
        world_frame = self.get_parameter("world_frame").get_parameter_value().string_value
        gps_topic = self.get_parameter("gps_topic").get_parameter_value().string_value
        goal_topic = self.get_parameter("goal_topic").get_parameter_value().string_value
        plan_topic = self.get_parameter("plan_topic").get_parameter_value().string_value

        if not self._api_key:
            self.get_logger().warn("amap_api_key 为空，请在配置文件中填入有效的高德 API Key")
        if not HAS_REQUESTS:
            self.get_logger().error("requests 库未安装，请执行: pip install requests")

        # ---- 状态 ----
        self._origin_lat = None   # 首个 GPS fix 作为投影原点
        self._origin_lon = None
        self._current_lat = None
        self._current_lon = None
        self._world_frame = world_frame  # 存储为实例变量，便于后续发布时使用

        # ---- 发布 / 订阅 ----
        self._plan_pub = self.create_publisher(Path, plan_topic, 10)
        self._gps_sub = self.create_subscription(
            NavSatFix, gps_topic, self._gps_callback, 10
        )
        self._goal_sub = self.create_subscription(
            PoseStamped, goal_topic, self._goal_callback, 10
        )

        self.get_logger().info(
            f"高德路径代理已启动 | "
            f"GPS话题={gps_topic} | 目标话题={goal_topic} | 路径话题={plan_topic}"
        )

    # ------------------------------------------------------------------

    def _gps_callback(self, msg: NavSatFix):
        self._current_lat = msg.latitude
        self._current_lon = msg.longitude
        if self._origin_lat is None:
            self._origin_lat = msg.latitude
            self._origin_lon = msg.longitude
            self.get_logger().info(
                f"投影原点已锁定: ({self._origin_lat:.6f}, {self._origin_lon:.6f})"
            )

    def _goal_callback(self, msg: PoseStamped):
        # 前置检查
        if self._current_lat is None:
            self.get_logger().error("尚未收到 GPS 定位 (/fix)，无法规划路径")
            return
        if not self._api_key or not HAS_REQUESTS:
            self.get_logger().error("API Key 或 requests 库未就绪，跳过规划")
            return

        # 终点：从 world 帧局部坐标反算 WGS-84
        goal_lat, goal_lon = _local_to_latlon(
            msg.pose.position.x, msg.pose.position.y,
            self._origin_lat, self._origin_lon,
        )

        try:
            path = self._request_walking_path(goal_lat, goal_lon)
        except Exception as e:
            self.get_logger().error(f"路径规划失败: {e}")
            return

        if path:
            self._plan_pub.publish(path)
            self.get_logger().info(
                f"\033[32m[SUCCESS] Gaode Path Received! "
                f"{len(path.poses)} points → /global_plan\033[0m"
            )
        else:
            self.get_logger().warn("API 返回空路径")

    # ------------------------------------------------------------------

    def _request_walking_path(self, goal_lat, goal_lon) -> Path | None:
        """调用高德步行路径规划 API，返回 nav_msgs/Path。"""

        # 起点: WGS-84 → GCJ-02（高德 API 要求火星坐标）
        origin_gcj_lng, origin_gcj_lat = wgs84_to_gcj02(
            self._current_lon, self._current_lat
        )
        goal_gcj_lng, goal_gcj_lat = wgs84_to_gcj02(goal_lon, goal_lat)

        params = {
            "origin": f"{origin_gcj_lng:.6f},{origin_gcj_lat:.6f}",
            "destination": f"{goal_gcj_lng:.6f},{goal_gcj_lat:.6f}",
            "key": self._api_key,
        }

        # 构造完整的 URL 用于调试透传
        debug_url = f"{AMAP_WALKING_URL}?origin={params['origin']}&destination={params['destination']}&key={self._api_key}"
        self.get_logger().info(
            f"[DEBUG] Amap Request: {debug_url}"
        )
        self.get_logger().debug(
            f"[DEBUG] Requesting Path: Start({self._current_lat:.6f},{self._current_lon:.6f}) "
            f"-> End({goal_lat:.6f},{goal_lon:.6f})"
        )

        try:
            resp = requests.get(AMAP_WALKING_URL, params=params, timeout=10.0)
            resp.raise_for_status()
            data = resp.json()
        except requests.exceptions.Timeout:
            self.get_logger().error("高德 API 请求超时（10s）")
            return None
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"高德 API 请求异常: {e}")
            return None
        except Exception as e:
            self.get_logger().error(f"API 响应解析失败: {e}")
            return None

        if data.get("status") != "1":
            self.get_logger().error(f"高德 API 返回异常: {data.get('info', '未知错误')}")
            return None

        # 提取路径点（遍历 steps 中每条 polyline）
        coords = []
        try:
            route = data["route"]
            paths = route.get("paths", [])
            if not paths:
                self.get_logger().warn("API 未返回任何路径方案")
                return None

            steps = paths[0].get("steps", [])
            for step in steps:
                poly = step.get("polyline", "")
                if not poly:
                    continue
                # 极简明文解析: "lon,lat;lon,lat;..."
                try:
                    points = poly.split(";")
                    for point_str in points:
                        lon_str, lat_str = point_str.split(",")
                        lon = float(lon_str.strip())
                        lat = float(lat_str.strip())
                        coords.append([lon, lat])
                except (ValueError, IndexError) as e:
                    self.get_logger().error(f"polyline 明文解析异常 (step skipped): {e}")
                    continue

            if not coords:
                self.get_logger().warn("[WARN] Destination is too close or path not found!")
                return None
        except (KeyError, TypeError) as e:
            self.get_logger().error(f"API 响应结构解析失败: {e}")
            return None

        # GCJ-02 → WGS-84 → 局部平面坐标
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self._world_frame  # 严格设置为 'world'

        for lng, lat in coords:
            try:
                wgs_lng, wgs_lat = gcj02_to_wgs84(lng, lat)
                x, y, _ = _latlon_to_local(
                    wgs_lat, wgs_lng, self._origin_lat, self._origin_lon, 0.0
                )
            except (OverflowError, ValueError) as e:
                self.get_logger().error(f"坐标转换失败 (point skipped): {e}")
                continue
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        if len(path_msg.poses) < 2:
            self.get_logger().warn("[WARN] Destination is too close or path not found!")
            return None

        # 成功解析，打印绿色成功日志
        self.get_logger().info(
            f"\033[32m[SUCCESS] Amap Path Parsed! Total points: {len(path_msg.poses)}\033[0m"
        )

        return path_msg


def main(args=None):
    rclpy.init(args=args)
    node = GaodePathProxy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
