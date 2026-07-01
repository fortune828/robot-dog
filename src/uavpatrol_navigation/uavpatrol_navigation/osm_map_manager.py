"""osm_map_manager.py — UAV patrol OSM 背景与语义先验节点

职责:
  1. 加载 OSM 地图，构建路网有向图
  2. RViz2 矢量可视化（路网 + 建筑）
  3. 提供建筑物等语义先验
  4. 可选启用 legacy road graph routing

坐标系: 所有输出 Marker 和 Path 均在 world 帧下 (ENU 米制)。
投影原点与 mock_gps_node 保持一致。
"""

import math
import os
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path as NavPath
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from uavpatrol_navigation.geo_utils import latlon_to_local, local_to_latlon

try:
    import osmnx as ox
    import networkx as nx
    HAS_OSMNX = True
except ImportError:
    HAS_OSMNX = False

try:
    from sanitation_interfaces.srv import PlanPath
    HAS_PLANPATH_SRV = True
except ImportError:
    HAS_PLANPATH_SRV = False


# ---------------------------------------------------------------------------
#  Node
# ---------------------------------------------------------------------------

class OsmMapManager(Node):
    """OSM 地图引擎 — 解析、可视化、离线路由、P2P 导航"""

    def __init__(self):
        super().__init__("osm_map_manager")

        if not HAS_OSMNX:
            self.get_logger().fatal("osmnx / networkx 未安装，请执行: pip install osmnx networkx")
            raise RuntimeError("osmnx and networkx are required")
        # ---- 参数 ----
        self.declare_parameter("map_file", "")
        self.declare_parameter("origin_lat", 30.747903)
        self.declare_parameter("origin_lon", 103.925269)
        self.declare_parameter("origin_from_first_fix", False)
        self.declare_parameter("enable_routing", False)
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("topology_marker_topic", "/osm_topology_markers")
        self.declare_parameter("plan_service_name", "/plan_osm_path")
        self.declare_parameter("goal_pose_topic", "/goal_pose")
        self.declare_parameter("global_plan_topic", "/global_plan")
        self.declare_parameter("gps_topic", "/fix")

        map_file = (
            self.get_parameter("map_file").get_parameter_value().string_value
        )
        self._origin_lat = (
            self.get_parameter("origin_lat").get_parameter_value().double_value
        )
        self._origin_lon = (
            self.get_parameter("origin_lon").get_parameter_value().double_value
        )
        self._origin_from_first_fix = (
            self.get_parameter("origin_from_first_fix")
            .get_parameter_value().bool_value
        )
        self._enable_routing = (
            self.get_parameter("enable_routing")
            .get_parameter_value().bool_value
        )
        self._origin_locked = not self._origin_from_first_fix
        self._world_frame = (
            self.get_parameter("world_frame").get_parameter_value().string_value
        )
        topology_topic = (
            self.get_parameter("topology_marker_topic")
            .get_parameter_value().string_value
        )
        plan_service = (
            self.get_parameter("plan_service_name")
            .get_parameter_value().string_value
        )
        goal_topic = (
            self.get_parameter("goal_pose_topic")
            .get_parameter_value().string_value
        )
        plan_topic = (
            self.get_parameter("global_plan_topic")
            .get_parameter_value().string_value
        )
        gps_topic = (
            self.get_parameter("gps_topic").get_parameter_value().string_value
        )

        # ---- 解析 map_file 路径 ----
        if not map_file:
            map_file = self._find_map_file()
            if not map_file:
                self.get_logger().fatal(
                    "未找到 map.osm，请将 map.osm 放入 data/ 或设置 map_file 参数"
                )
                raise FileNotFoundError("map.osm not found")

        self._map_file = map_file
        self.get_logger().info(f"Loading OSM map: {map_file}")
        if self._origin_from_first_fix:
            self.get_logger().info(
                "OSM ENU origin will be replaced by the first RTK fix"
            )

        # ---- GPS 状态 ----
        self._current_lat = None
        self._current_lon = None

        # ---- 加载 & 构图 ----
        self._graph = self._load_road_graph(map_file)
        self._buildings = self._load_buildings(map_file)

        # ---- 预构建 & 定时心跳发布拓扑 Marker ----
        self._marker_array = self._build_topology_markers()
        self._topo_pub = self.create_publisher(
            MarkerArray, topology_topic, 10
        )
        self._publish_topology_markers()  # 立即首发
        self.create_timer(3.0, self._publish_topology_markers)  # 每 3s 心跳

        # ---- 可选离线路由服务（UAV 主线默认禁用，不沿 road graph 飞行）----
        self._srv = None
        if self._enable_routing:
            if not HAS_PLANPATH_SRV:
                self.get_logger().fatal(
                    "PlanPath srv 未生成，请先 colcon build sanitation_interfaces"
                )
                raise RuntimeError("PlanPath srv not available")
            self._srv = self.create_service(
                PlanPath, plan_service, self._plan_path_callback
            )

        # ---- GPS 订阅（用于 P2P 导航起点） ----
        self._gps_sub = self.create_subscription(
            NavSatFix, gps_topic, self._gps_callback, 10
        )

        # ---- 可选 P2P road graph 导航订阅（默认禁用）----
        self._goal_sub = None
        if self._enable_routing:
            self._goal_sub = self.create_subscription(
                PoseStamped, goal_topic, self._goal_pose_callback, 10
            )

        # ---- 全局路径发布 ----
        self._plan_pub = self.create_publisher(NavPath, plan_topic, 10)

        self.get_logger().info(
            f"OSM Map Manager 已启动 | "
            f"nodes={len(self._graph.nodes)} edges={len(self._graph.edges)} | "
            f"buildings={len(self._buildings)} | "
            f"routing_enabled={self._enable_routing}"
        )

    @staticmethod
    def _find_map_file():
        roots = []
        for env_name in ("UAVPATROL_ROOT", "ROBOTDOG_ROOT"):
            env_value = os.environ.get(env_name)
            if env_value:
                roots.append(Path(env_value))
        roots.append(Path.cwd())
        p = Path(__file__).resolve()
        roots.extend([p] + list(p.parents))

        seen = set()
        for root in roots:
            root = root.resolve()
            if root in seen:
                continue
            seen.add(root)
            candidate = root / "data" / "map.osm"
            if candidate.is_file():
                return str(candidate)
        return ""

    # ------------------------------------------------------------------
    #  GPS
    # ------------------------------------------------------------------

    def _gps_callback(self, msg: NavSatFix):
        self._current_lat = msg.latitude
        self._current_lon = msg.longitude
        if not self._origin_from_first_fix or self._origin_locked:
            return
        if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            self.get_logger().warn("忽略无效 GPS，暂不锁定 OSM 投影原点")
            return

        self._origin_lat = msg.latitude
        self._origin_lon = msg.longitude
        self._origin_locked = True
        self.get_logger().info(
            f"OSM ENU 原点已由第一帧 RTK 锁定: "
            f"({self._origin_lat:.8f}, {self._origin_lon:.8f})"
        )
        self._buildings = self._load_buildings(self._map_file)
        self._marker_array = self._build_topology_markers()
        self._publish_topology_markers()

    # ------------------------------------------------------------------
    #  Map loading
    # ------------------------------------------------------------------

    def _load_road_graph(self, map_file):
        """使用 osmnx 加载路网，构建有向图 (MultiDiGraph)。

        Node 属性: x=lon, y=lat
        Edge 属性: length (m), geometry (可选的中间点序列)
        """
        self.get_logger().info("Parsing road network with osmnx ...")
        G = ox.graph_from_xml(map_file)
        self.get_logger().info(
            f"Road graph: {len(G.nodes)} nodes, {len(G.edges)} edges"
        )
        return G

    def _load_buildings(self, map_file):
        """从 OSM XML 中提取建筑多边形。

        手动解析 <node> 和 <way building=*> 元素，避免引入 geopandas 等重依赖。
        返回 list of list of (x, y) — 每组坐标已在 ENU 下且闭合。
        """
        self.get_logger().info("Extracting buildings from OSM XML ...")
        buildings = []
        try:
            tree = ET.parse(map_file)
            root = tree.getroot()

            # Pass 1: 收集所有 node → (lat, lon)
            nodes = {}
            for node in root.findall("node"):
                node_id = node.get("id")
                lat = float(node.get("lat"))
                lon = float(node.get("lon"))
                nodes[node_id] = (lat, lon)

            # Pass 2: 遍历 way，筛选 building 标签
            for way in root.findall("way"):
                is_building = False
                for tag in way.findall("tag"):
                    k = tag.get("k", "")
                    v = tag.get("v", "")
                    if k == "building" and v not in (
                        "no", "demolished", "dismantled", "ruins",
                    ):
                        is_building = True
                        break
                if not is_building:
                    continue

                nds = way.findall("nd")
                if len(nds) < 3:
                    continue

                # 解析坐标并投影到 ENU
                coords = []
                for nd in nds:
                    ref = nd.get("ref")
                    if ref in nodes:
                        lat, lon = nodes[ref]
                        x, y, _ = latlon_to_local(
                            lat, lon, self._origin_lat, self._origin_lon,
                        )
                        coords.append((x, y))

                if len(coords) >= 3:
                    if coords[0] != coords[-1]:
                        coords.append(coords[0])
                    buildings.append(coords)

            self.get_logger().info(
                f"Extracted {len(buildings)} building outlines"
            )
        except Exception as e:
            self.get_logger().error(f"建筑提取失败: {e}")

        return buildings

    # ------------------------------------------------------------------
    #  RViz2 可视化
    # ------------------------------------------------------------------

    def _build_topology_markers(self):
        """预构建 MarkerArray（一次性），返回后由心跳定时器重复发布。"""
        marker_array = MarkerArray()
        marker_id = 0

        # ---- 路网: 荧光绿 LINE_STRIP (线宽 0.8m) ----
        for u, v, key, data in self._graph.edges(keys=True, data=True):
            points = []
            if "geometry" in data:
                for lon, lat in data["geometry"].coords:
                    x, y, _ = latlon_to_local(
                        lat, lon, self._origin_lat, self._origin_lon,
                    )
                    points.append(Point(x=x, y=y, z=0.0))
            else:
                u_nd = self._graph.nodes[u]
                v_nd = self._graph.nodes[v]
                x1, y1, _ = latlon_to_local(
                    u_nd["y"], u_nd["x"], self._origin_lat, self._origin_lon,
                )
                x2, y2, _ = latlon_to_local(
                    v_nd["y"], v_nd["x"], self._origin_lat, self._origin_lon,
                )
                points.append(Point(x=x1, y=y1, z=0.0))
                points.append(Point(x=x2, y=y2, z=0.0))

            if len(points) >= 2:
                marker = Marker()
                marker.header.frame_id = self._world_frame
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "roads"
                marker.id = marker_id
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.8
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
                marker.points = points
                marker_array.markers.append(marker)
                marker_id += 1

        # ---- 建筑: 浅蓝色 LINE_STRIP (线宽 0.8m, alpha=0.5) ----
        for coords in self._buildings:
            points = [Point(x=x, y=y, z=0.0) for x, y in coords]
            if len(points) >= 3:
                marker = Marker()
                marker.header.frame_id = self._world_frame
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "buildings"
                marker.id = marker_id
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.8
                marker.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.5)
                marker.points = points
                marker_array.markers.append(marker)
                marker_id += 1

        self.get_logger().info(
            f"Topology markers built: {marker_id} total "
            f"(roads + {len(self._buildings)} buildings)"
        )
        return marker_array

    def _publish_topology_markers(self):
        """定时心跳发布 — 刷新 timestamp 后广播预构建的 MarkerArray。"""
        if self._marker_array is None:
            return
        now = self.get_clock().now().to_msg()
        for marker in self._marker_array.markers:
            marker.header.stamp = now
        self._topo_pub.publish(self._marker_array)

    # ------------------------------------------------------------------
    #  Routing core
    # ------------------------------------------------------------------

    def _find_nearest_node(self, lat, lon):
        """在路网图中找到最近的 node。

        优先使用 osmnx 内置 KD-tree；回退到暴力扫描。
        """
        try:
            return ox.distance.nearest_nodes(self._graph, X=lon, Y=lat)
        except Exception:
            best_id = None
            best_dist = float("inf")
            for nid, ndata in self._graph.nodes(data=True):
                dx = ndata["x"] - lon
                dy = ndata["y"] - lat
                d = dx * dx + dy * dy
                if d < best_dist:
                    best_dist = d
                    best_id = nid
            return best_id

    def _plan_route(self, start_x, start_y, goal_x, goal_y):
        """核心路由：ENU 坐标 → WGS-84 → 最近邻节点 → 最短路径 → ENU。

        Returns:
            (list_of_(x,y), success: bool, message: str)
        """
        # ENU → WGS-84
        start_lat, start_lon = local_to_latlon(
            start_x, start_y, self._origin_lat, self._origin_lon,
        )
        goal_lat, goal_lon = local_to_latlon(
            goal_x, goal_y, self._origin_lat, self._origin_lon,
        )

        # 最近邻节点
        src = self._find_nearest_node(start_lat, start_lon)
        tgt = self._find_nearest_node(goal_lat, goal_lon)

        if src is None or tgt is None:
            return [], False, "无法找到最近的图节点"

        if src == tgt:
            return (
                [(start_x, start_y), (goal_x, goal_y)],
                True,
                "start == goal — trivial path",
            )

        # Dijkstra / A* 最短路径（weight='length' 米）
        try:
            path_nodes = nx.shortest_path(
                self._graph, source=src, target=tgt, weight="length",
            )
        except nx.NetworkXNoPath:
            return [], False, f"无连通路径: {src} → {tgt}"
        except Exception as e:
            return [], False, f"路径搜索异常: {e}"

        # 保留道路边 geometry 中的弯道点，避免节点间直线穿出道路。
        path_enu = [(start_x, start_y)]
        for u, v in zip(path_nodes, path_nodes[1:]):
            edge_group = self._graph.get_edge_data(u, v) or {}
            if not edge_group:
                continue
            edge = min(
                edge_group.values(),
                key=lambda data: data.get("length", float("inf")),
            )
            if "geometry" in edge:
                coords = list(edge["geometry"].coords)
                u_node = self._graph.nodes[u]
                first_dist = (coords[0][0] - u_node["x"]) ** 2 + (coords[0][1] - u_node["y"]) ** 2
                last_dist = (coords[-1][0] - u_node["x"]) ** 2 + (coords[-1][1] - u_node["y"]) ** 2
                if last_dist < first_dist:
                    coords.reverse()
            else:
                coords = [
                    (self._graph.nodes[u]["x"], self._graph.nodes[u]["y"]),
                    (self._graph.nodes[v]["x"], self._graph.nodes[v]["y"]),
                ]

            for lon, lat in coords:
                point = latlon_to_local(lat, lon, self._origin_lat, self._origin_lon)[:2]
                if math.hypot(point[0] - path_enu[-1][0], point[1] - path_enu[-1][1]) > 0.01:
                    path_enu.append(point)

        if math.hypot(goal_x - path_enu[-1][0], goal_y - path_enu[-1][1]) > 0.01:
            path_enu.append((goal_x, goal_y))

        return path_enu, True, f"{len(path_enu)} nodes"

    # ------------------------------------------------------------------
    #  Routing service: /plan_osm_path
    # ------------------------------------------------------------------

    def _plan_path_callback(self, request, response):
        """服务回调：ENU 起点 → ENU 终点 → nav_msgs/Path"""
        path_enu, success, message = self._plan_route(
            request.start_x, request.start_y,
            request.goal_x, request.goal_y,
        )

        response.success = success
        response.message = message

        path_msg = NavPath()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self._world_frame

        for x, y in path_enu:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        response.path = path_msg

        if success:
            self.get_logger().info(
                f"/plan_osm_path: {message} | "
                f"({request.start_x:.1f},{request.start_y:.1f}) → "
                f"({request.goal_x:.1f},{request.goal_y:.1f})"
            )
        else:
            self.get_logger().warn(f"/plan_osm_path failed: {message}")

        return response

    # ------------------------------------------------------------------
    #  P2P navigation: /goal_pose → /global_plan
    # ------------------------------------------------------------------

    def _goal_pose_callback(self, msg: PoseStamped):
        """独立 P2P 导航：收到目标点 → 路由 → 发布全局路径。"""
        if self._current_lat is None or self._current_lon is None:
            self.get_logger().warn("GPS 未就绪，无法执行 P2P 导航")
            return

        cur_x, cur_y, _ = latlon_to_local(
            self._current_lat, self._current_lon,
            self._origin_lat, self._origin_lon,
        )
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y

        dist = math.hypot(goal_x - cur_x, goal_y - cur_y)
        if dist < 1.0:
            self.get_logger().info(f"P2P target within {dist:.1f}m, skipped")
            return

        path_enu, success, message = self._plan_route(cur_x, cur_y, goal_x, goal_y)
        if not success:
            self.get_logger().error(f"P2P routing failed: {message}")
            return

        self._publish_path(path_enu)
        self.get_logger().info(
            f"\033[32m[SUCCESS] P2P route: {len(path_enu)} nodes | "
            f"({cur_x:.1f},{cur_y:.1f}) → ({goal_x:.1f},{goal_y:.1f})\033[0m"
        )

    # ------------------------------------------------------------------
    #  Helpers
    # ------------------------------------------------------------------

    def _publish_path(self, points):
        """将 (x, y) 列表发布为 nav_msgs/Path (world 帧)。"""
        path_msg = NavPath()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self._world_frame
        for px, py in points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self._plan_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OsmMapManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
