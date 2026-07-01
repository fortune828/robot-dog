"""polygon_coverage_planner.py — UAV 区域空域覆盖路径规划节点

Architecture:
  patrol_area.json → polygon_coverage_planner → /global_plan
  /clicked_point (RViz Publish Point) → polygon_coverage_planner → /global_plan
  /fix (NavSatFix) → polygon_coverage_planner (GPS 感知)

UAV 主线默认不沿 OSM road graph 飞行。OSM 只作为地图背景和语义先验来源。
默认链路是: 巡逻区域 polygon → 空域覆盖规划 → /global_plan。

用户通过 RViz Publish Point 工具点击多边形顶点，当新点与首点距离 < closing_threshold
时触发闭合并生成最优角度扫掠线弓字形覆盖路径。
"""

import heapq
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path as NavPath
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

try:
    from shapely.geometry import Polygon, LineString
    from shapely import affinity
    HAS_SHAPELY = True
except ImportError:
    HAS_SHAPELY = False

from uavpatrol_navigation.geo_utils import latlon_to_local
from uavpatrol_navigation.patrol_area_io import (
    load_patrol_area,
    patrol_area_to_local_vertices,
)
from uavpatrol_navigation.planning_defaults import DEFAULT_PLANNING
from uavpatrol_navigation.semantic_coverage_planner import plan_semantic_coverage

try:
    from sanitation_interfaces.srv import PlanPath
    HAS_PLANPATH_SRV = True
except ImportError:
    HAS_PLANPATH_SRV = False


def _safe_connector(poly, start, end):
    """Return a shortest visible connector that remains inside the polygon."""
    direct = LineString([start, end])
    if poly.covers(direct):
        return [start, end]

    vertices = [start, end]
    for ring in [poly.exterior, *poly.interiors]:
        vertices.extend(list(ring.coords)[:-1])

    graph = [[] for _ in vertices]
    for i in range(len(vertices)):
        for j in range(i + 1, len(vertices)):
            segment = LineString([vertices[i], vertices[j]])
            if poly.covers(segment):
                distance = segment.length
                graph[i].append((distance, j))
                graph[j].append((distance, i))

    queue = [(0.0, 0)]
    distances = {0: 0.0}
    previous = {}
    while queue:
        distance, node = heapq.heappop(queue)
        if node == 1:
            break
        if distance != distances.get(node):
            continue
        for edge_length, neighbor in graph[node]:
            candidate = distance + edge_length
            if candidate < distances.get(neighbor, float("inf")):
                distances[neighbor] = candidate
                previous[neighbor] = node
                heapq.heappush(queue, (candidate, neighbor))

    if 1 not in distances:
        return None
    route = [1]
    while route[-1] != 0:
        route.append(previous[route[-1]])
    route.reverse()
    return [vertices[index] for index in route]


class PolygonCoveragePlanner(Node):
    """全覆盖路径规划节点 — UAV 区域 polygon 到覆盖航点"""

    def __init__(self):
        super().__init__("polygon_coverage_planner")

        if not HAS_SHAPELY:
            self.get_logger().fatal("shapely 未安装，请执行: pip install shapely")
            raise RuntimeError("shapely is required for CPP polygon operations")

        # ---- 参数 ----
        self.declare_parameter("clicked_point_topic", "/clicked_point")
        self.declare_parameter("area_source", "file")  # file / clicked_point / demo
        self.declare_parameter("patrol_area_file", DEFAULT_PLANNING.patrol_area_file)
        self.declare_parameter("plan_topic", "/global_plan")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("sweep_spacing", 1.5)
        self.declare_parameter("closing_threshold", 1.0)
        self.declare_parameter("allow_outside_connectors", True)
        self.declare_parameter("wrap_coverage_from_entry", True)
        self.declare_parameter("gps_topic", "/fix")
        self.declare_parameter("mission_status_topic", "/mission_status")
        self.declare_parameter("osm_plan_service", "/plan_osm_path")
        self.declare_parameter("goal_pose_topic", "/goal_pose")
        self.declare_parameter("use_osm_transit", False)
        self.declare_parameter("repeat_on_mission_complete", False)
        self.declare_parameter("enable_osm_semantic_constraints", True)
        self.declare_parameter("map_file", DEFAULT_PLANNING.map_file)
        self.declare_parameter("output_dir", DEFAULT_PLANNING.route_output_dir)
        self.declare_parameter("building_safety_buffer_m", DEFAULT_PLANNING.building_safety_buffer_m)
        self.declare_parameter("default_building_height_m", DEFAULT_PLANNING.default_building_height_m)
        self.declare_parameter("avoid_building_overflight", True)
        self.declare_parameter("min_flyable_area_m2", DEFAULT_PLANNING.min_flyable_area_m2)
        self.declare_parameter("coverage_spacing_m", DEFAULT_PLANNING.coverage_spacing_m)
        self.declare_parameter("route_edge_margin_m", DEFAULT_PLANNING.route_edge_margin_m)
        self.declare_parameter("max_off_area_distance_m", DEFAULT_PLANNING.max_off_area_distance_m)
        self.declare_parameter("sensor_coverage_radius_m", DEFAULT_PLANNING.sensor_coverage_radius_m)
        self.declare_parameter("min_coverage_contribution_m", DEFAULT_PLANNING.min_coverage_contribution_m)
        self.declare_parameter("max_connection_length_m", DEFAULT_PLANNING.max_connection_length_m)
        self.declare_parameter("simplify_tolerance_m", DEFAULT_PLANNING.simplify_tolerance_m)
        self.declare_parameter("max_2opt_iterations", DEFAULT_PLANNING.max_2opt_iterations)
        self.declare_parameter("disable_2opt_if_strokes_gt", DEFAULT_PLANNING.disable_2opt_if_strokes_gt)

        clicked_topic = (
            self.get_parameter("clicked_point_topic")
            .get_parameter_value().string_value
        )
        self._area_source = (
            self.get_parameter("area_source")
            .get_parameter_value().string_value
            .strip()
            .lower()
        )
        self._patrol_area_file = (
            self.get_parameter("patrol_area_file")
            .get_parameter_value().string_value
        )
        plan_topic = (
            self.get_parameter("plan_topic")
            .get_parameter_value().string_value
        )
        self._world_frame = (
            self.get_parameter("world_frame")
            .get_parameter_value().string_value
        )
        self._sweep_spacing = (
            self.get_parameter("sweep_spacing")
            .get_parameter_value().double_value
        )
        self._closing_threshold = (
            self.get_parameter("closing_threshold")
            .get_parameter_value().double_value
        )
        self._allow_outside_connectors = (
            self.get_parameter("allow_outside_connectors")
            .get_parameter_value().bool_value
        )
        self._wrap_coverage_from_entry = (
            self.get_parameter("wrap_coverage_from_entry")
            .get_parameter_value().bool_value
        )
        if self._sweep_spacing <= 0.0 or self._closing_threshold <= 0.0:
            raise ValueError("sweep_spacing and closing_threshold must be positive")
        if self._area_source not in ("file", "clicked_point", "demo"):
            raise ValueError("area_source must be file, clicked_point, or demo")
        gps_topic = (
            self.get_parameter("gps_topic")
            .get_parameter_value().string_value
        )
        mission_status_topic = (
            self.get_parameter("mission_status_topic")
            .get_parameter_value().string_value
        )
        osm_plan_service = (
            self.get_parameter("osm_plan_service")
            .get_parameter_value().string_value
        )
        goal_pose_topic = (
            self.get_parameter("goal_pose_topic")
            .get_parameter_value().string_value
        )
        self._use_osm_transit = (
            self.get_parameter("use_osm_transit")
            .get_parameter_value().bool_value
        )
        self._repeat_on_mission_complete = (
            self.get_parameter("repeat_on_mission_complete")
            .get_parameter_value().bool_value
        )
        self._enable_osm_semantic_constraints = (
            self.get_parameter("enable_osm_semantic_constraints")
            .get_parameter_value().bool_value
        )
        self._map_file = self.get_parameter("map_file").get_parameter_value().string_value
        self._output_dir = self.get_parameter("output_dir").get_parameter_value().string_value
        self._building_safety_buffer_m = (
            self.get_parameter("building_safety_buffer_m")
            .get_parameter_value().double_value
        )
        self._default_building_height_m = (
            self.get_parameter("default_building_height_m")
            .get_parameter_value().double_value
        )
        self._avoid_building_overflight = (
            self.get_parameter("avoid_building_overflight")
            .get_parameter_value().bool_value
        )
        self._min_flyable_area_m2 = (
            self.get_parameter("min_flyable_area_m2")
            .get_parameter_value().double_value
        )
        self._coverage_spacing_m = (
            self.get_parameter("coverage_spacing_m")
            .get_parameter_value().double_value
        )
        self._route_edge_margin_m = (
            self.get_parameter("route_edge_margin_m")
            .get_parameter_value().double_value
        )
        self._max_off_area_distance_m = (
            self.get_parameter("max_off_area_distance_m")
            .get_parameter_value().double_value
        )
        self._sensor_coverage_radius_m = (
            self.get_parameter("sensor_coverage_radius_m")
            .get_parameter_value().double_value
        )
        self._min_coverage_contribution_m = (
            self.get_parameter("min_coverage_contribution_m")
            .get_parameter_value().double_value
        )
        self._max_connection_length_m = (
            self.get_parameter("max_connection_length_m")
            .get_parameter_value().double_value
        )
        self._simplify_tolerance_m = (
            self.get_parameter("simplify_tolerance_m")
            .get_parameter_value().double_value
        )
        self._max_2opt_iterations = (
            self.get_parameter("max_2opt_iterations")
            .get_parameter_value().integer_value
        )
        self._disable_2opt_if_strokes_gt = (
            self.get_parameter("disable_2opt_if_strokes_gt")
            .get_parameter_value().integer_value
        )

        # ---- 状态 ----
        self._vertices = []             # list of (x, y) — 局部 ENU 坐标 (米)
        self.current_cpp_points = []    # 当前场地弓字形点阵 (内存)
        self._file_area_data = None
        self._file_area_loaded = False
        self._file_area_dispatched = False

        # ---- GPS 状态 ----
        self._origin_lat = None
        self._origin_lon = None
        self._current_lat = None
        self._current_lon = None

        # ---- 异步通勤状态 ----
        self._pending_cpp_points = []   # 等待通勤响应期间的 CPP 点阵
        self._pending_min_idx = 0       # 等待通勤响应期间的接入索引
        self._request_generation = 0    # 隔离过期的异步 OSM 响应

        # ---- OSM 离线路由客户端（默认禁用；UAV 不沿 road graph 飞行）----
        self._osm_client = None
        if self._use_osm_transit:
            if not HAS_PLANPATH_SRV:
                self.get_logger().fatal(
                    "PlanPath srv 未生成，请先 colcon build sanitation_interfaces"
                )
                raise RuntimeError("PlanPath srv not available")
            self._osm_client = self.create_client(PlanPath, osm_plan_service)
            if not self._osm_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(
                    f"{osm_plan_service} 暂未就绪；规划时将降级为直接下发覆盖路径"
                )

        # ---- 发布 / 订阅 ----
        self._plan_pub = self.create_publisher(NavPath, plan_topic, 10)
        self._clicked_sub = None
        if self._area_source in ("clicked_point", "demo"):
            self._clicked_sub = self.create_subscription(
                PointStamped, clicked_topic, self._clicked_callback, 10
            )
        self._gps_sub = self.create_subscription(
            NavSatFix, gps_topic, self._gps_callback, 10
        )
        self._mission_sub = self.create_subscription(
            String, mission_status_topic, self._mission_status_callback, 10
        )
        self._goal_pose_sub = None
        if self._area_source != "file":
            self._goal_pose_sub = self.create_subscription(
                PoseStamped, goal_pose_topic, self._goal_pose_callback, 10
            )

        if self._area_source == "file":
            self._load_file_area()

        self.get_logger().info(
            f"CPP Planner 已启动 (事件驱动) | sweep_spacing={self._sweep_spacing}m | "
            f"closing_threshold={self._closing_threshold}m | "
            f"allow_outside_connectors={self._allow_outside_connectors} | "
            f"wrap_coverage_from_entry={self._wrap_coverage_from_entry} | "
            f"area_source={self._area_source} | "
            f"patrol_area_file={self._patrol_area_file} | "
            f"clicked_point={clicked_topic} | GPS on {gps_topic} | "
            f"mission_status on {mission_status_topic} | "
            f"use_osm_transit={self._use_osm_transit} | "
            f"semantic_constraints={self._enable_osm_semantic_constraints}"
        )

    # ------------------------------------------------------------------
    #  文件巡逻区域输入
    # ------------------------------------------------------------------

    def _load_file_area(self):
        try:
            data, path, frame = load_patrol_area(self._patrol_area_file)
        except Exception as exc:
            self.get_logger().error(f"加载 patrol area 失败: {exc}")
            return
        self._file_area_data = data
        self._file_area_loaded = True
        self.get_logger().info(
            f"Loaded patrol area from {path} | "
            f"name={data.get('area_name', 'patrol_area')} | frame={frame} | "
            f"vertices={len(data.get('boundary', []))}"
        )
        self._try_dispatch_file_area()

    def _try_dispatch_file_area(self):
        if (
            self._area_source != "file"
            or not self._file_area_loaded
            or self._file_area_dispatched
        ):
            return
        frame = str(self._file_area_data.get("coordinate_frame", "WGS84")).upper()
        if frame in ("WGS84", "CGCS2000") and self._origin_lat is None:
            self.get_logger().info("Waiting for first /fix before projecting patrol_area.json")
            return
        origin_lat = self._origin_lat if self._origin_lat is not None else 0.0
        origin_lon = self._origin_lon if self._origin_lon is not None else 0.0
        self._file_area_dispatched = True
        if self._enable_osm_semantic_constraints and self._avoid_building_overflight:
            try:
                semantic = plan_semantic_coverage(
                    map_file=self._map_file,
                    patrol_area_file=self._patrol_area_file,
                    output_dir=self._output_dir,
                    origin_lat=origin_lat,
                    origin_lon=origin_lon,
                    building_safety_buffer_m=self._building_safety_buffer_m,
                    default_building_height_m=self._default_building_height_m,
                    min_flyable_area_m2=self._min_flyable_area_m2,
                    coverage_spacing_m=(
                        self._coverage_spacing_m
                        if self._coverage_spacing_m > 0.0
                        else self._sweep_spacing
                    ),
                    route_edge_margin_m=self._route_edge_margin_m,
                    max_off_area_distance_m=self._max_off_area_distance_m,
                    sensor_coverage_radius_m=self._sensor_coverage_radius_m,
                    min_coverage_contribution_m=self._min_coverage_contribution_m,
                    max_connection_length_m=self._max_connection_length_m,
                    simplify_tolerance_m=self._simplify_tolerance_m,
                    max_2opt_iterations=self._max_2opt_iterations,
                    disable_2opt_if_strokes_gt=self._disable_2opt_if_strokes_gt,
                    write_outputs=True,
                )
                if len(semantic.route_local) >= 2:
                    self.get_logger().info(
                        "[Semantic CPP] "
                        f"obstacles={len(semantic.static_obstacles_local)} | "
                        f"buffers={len(semantic.obstacle_buffers_local)} | "
                        f"route_points={len(semantic.route_local)}"
                    )
                    self._generate_and_publish_points(semantic.route_local)
                    return
                self.get_logger().warn(
                    "Semantic CPP produced fewer than 2 points; falling back to plain coverage"
                )
            except Exception as exc:
                self.get_logger().error(
                    f"Semantic CPP failed, falling back to plain coverage: {exc}"
                )

        try:
            vertices = patrol_area_to_local_vertices(
                self._file_area_data, origin_lat, origin_lon
            )
        except Exception as exc:
            self.get_logger().error(f"patrol area 投影失败: {exc}")
            return
        self.get_logger().info(
            f"[CPP] Dispatching file patrol area with {len(vertices)} vertices"
        )
        self._generate_and_publish(vertices)

    # ------------------------------------------------------------------
    #  GPS 订阅
    # ------------------------------------------------------------------

    def _gps_callback(self, msg: NavSatFix):
        self._current_lat = msg.latitude
        self._current_lon = msg.longitude
        if self._origin_lat is None:
            self._origin_lat = msg.latitude
            self._origin_lon = msg.longitude
            self.get_logger().info(
                f"CPP 投影原点已锁定: ({self._origin_lat:.6f}, {self._origin_lon:.6f})"
            )
            self._try_dispatch_file_area()

    # ------------------------------------------------------------------
    #  多边形顶点收集 (RViz Publish Point)
    # ------------------------------------------------------------------

    def _clicked_callback(self, msg: PointStamped):
        """收集 RViz Publish Point 工具点击的点。

        /clicked_point 在 world 固定帧下给出局部 ENU 坐标 (x, y)，
        直接存入顶点列表。当新点与首点距离 < closing_threshold 时触发闭合。
        """
        x = msg.point.x
        y = msg.point.y

        if not self._vertices:
            self._vertices.append((x, y))
            self.get_logger().info(
                f"\033[36m[CPP] First vertex: ({x:.2f}, {y:.2f})\033[0m"
            )
            return

        dist_to_first = math.hypot(
            x - self._vertices[0][0], y - self._vertices[0][1]
        )

        if dist_to_first < self._closing_threshold and len(self._vertices) >= 3:
            self.get_logger().info(
                f"\033[36m[CPP] Polygon closed! "
                f"{len(self._vertices)} vertices → generating coverage path...\033[0m"
            )

            vertices = list(self._vertices)
            self._vertices.clear()
            self._generate_and_publish(vertices)
        else:
            self._vertices.append((x, y))
            self.get_logger().info(
                f"\033[36m[CPP] Vertex {len(self._vertices)}: "
                f"({x:.2f}, {y:.2f}) | dist_to_close={dist_to_first:.2f}m\033[0m"
            )

    # ------------------------------------------------------------------
    #  任务状态回调 (事件驱动 — 无限折返)
    # ------------------------------------------------------------------

    def _mission_status_callback(self, msg: String):
        if not self._repeat_on_mission_complete:
            return
        if msg.data == "REACHED_GOAL":
            if not self.current_cpp_points:
                return  # 非 CPP 业务触发的到达，忽略
            self.get_logger().info(
                "\033[35m[INFO] Reversing coverage path and restarting patrol...\033[0m"
            )
            self.current_cpp_points.reverse()
            self._publish_path(self.current_cpp_points)

    # ------------------------------------------------------------------
    #  手动导航打断 (跨节点失忆)
    # ------------------------------------------------------------------

    def _goal_pose_callback(self, msg: PoseStamped):
        """监听到 /goal_pose → 用户正在使用 P2P 导航，清空 CPP 记忆。"""
        if self.current_cpp_points or self._pending_cpp_points:
            self.current_cpp_points = []
            self._pending_cpp_points = []
            self._request_generation += 1
            self.get_logger().warn(
                "\033[33m[WARN] Manual navigation detected. "
                "Clearing patrol memory.\033[0m"
            )

    # ------------------------------------------------------------------
    #  路径生成与发布
    # ------------------------------------------------------------------

    def _generate_and_publish(self, vertices):
        """生成弓字形覆盖路径，最近接入点寻优 + 切片下发。

        Flow:
          1. Shapely 生成完整 CPP 点阵 → 存入 current_cpp_points (完整)
          2. 找当前位置到 CPP 中所有点的最近点索引 min_idx
          3. 默认直接发布 CPP[min_idx:]，不使用 OSM road graph
          4. 内存保留完整 CPP，供后续折返使用
        """
        cpp_points = self._compute_coverage_path(vertices)
        if cpp_points is None or len(cpp_points) < 2:
            return
        self._generate_and_publish_points(cpp_points)

    def _generate_and_publish_points(self, cpp_points):
        """Dispatch an already generated local ENU coverage point sequence."""
        self.current_cpp_points = cpp_points

        # ---- 最近接入点寻优 ----
        min_idx = 0
        if self._current_lat is not None and self._origin_lat is not None:
            cur_x, cur_y, _ = latlon_to_local(
                self._current_lat, self._current_lon,
                self._origin_lat, self._origin_lon, 0.0,
            )
            min_dist = float("inf")
            for i, (px, py) in enumerate(cpp_points):
                d = math.hypot(px - cur_x, py - cur_y)
                if d < min_dist:
                    min_dist = d
                    min_idx = i
            self.get_logger().info(
                f"[CPP] Nearest entry: idx={min_idx}/{len(cpp_points)}, "
                f"dist={min_dist:.1f}m"
            )
        else:
            self.get_logger().warn("GPS 未就绪，默认以 CPP[0] 为接入点")

        entry_x, entry_y = cpp_points[min_idx]

        # ---- 存储待定状态（回调中拼接） ----
        self._request_generation += 1
        generation = self._request_generation
        self._pending_cpp_points = cpp_points
        self._pending_min_idx = min_idx

        if self._use_osm_transit:
            self._request_transit_path_async(entry_x, entry_y, generation)
        else:
            self._dispatch_without_transit(generation)

    # ------------------------------------------------------------------
    #  OSM 异步通勤路径请求
    # ------------------------------------------------------------------

    def _request_transit_path_async(self, dest_x, dest_y, generation):
        """异步调用 /plan_osm_path 获取从当前位置到作业起点的拓扑路径。

        稳健性降级:
          - GPS 未就绪 → 直接下发纯 CPP
          - 距离 < 10m → 跳过通勤
          - 服务调用失败 → 跳过通勤

        结果由 _on_transit_response 异步处理。
        """
        if self._current_lat is None or self._origin_lat is None:
            self.get_logger().warn("GPS 未就绪，跳过通勤段，直接下发 CPP")
            self._dispatch_without_transit(generation)
            return

        cur_x, cur_y, _ = latlon_to_local(
            self._current_lat, self._current_lon,
            self._origin_lat, self._origin_lon, 0.0,
        )

        dist = math.hypot(dest_x - cur_x, dest_y - cur_y)
        if dist < 10.0:
            self.get_logger().info(
                f"距离作业起点 {dist:.1f}m < 10m，跳过通勤段"
            )
            self._dispatch_without_transit(generation)
            return

        if self._osm_client is None or not self._osm_client.service_is_ready():
            self.get_logger().warn("OSM 路由服务未就绪，直接下发 CPP")
            self._dispatch_without_transit(generation)
            return

        req = PlanPath.Request()
        req.start_x = cur_x
        req.start_y = cur_y
        req.goal_x = dest_x
        req.goal_y = dest_y

        self.get_logger().info(
            f"[OSM Transit] Requesting path: "
            f"({cur_x:.1f},{cur_y:.1f}) → ({dest_x:.1f},{dest_y:.1f})"
        )

        future = self._osm_client.call_async(req)
        future.add_done_callback(
            lambda done_future: self._on_transit_response(done_future, generation)
        )

    def _on_transit_response(self, future, generation):
        """OSM 通勤路径异步回调：拼接 Transit + CPP[min_idx:] 并下发。"""
        if generation != self._request_generation:
            self.get_logger().warn("忽略已过期的 OSM 通勤响应")
            return
        cpp_points = self._pending_cpp_points
        min_idx = self._pending_min_idx

        if not cpp_points:
            return  # 状态已被清除

        transit_points = []
        try:
            response = future.result()
            if response.success and response.path.poses:
                for pose in response.path.poses:
                    transit_points.append(
                        (pose.pose.position.x, pose.pose.position.y)
                    )
                self.get_logger().info(
                    f"\033[32m[SUCCESS] OSM Transit: "
                    f"{len(transit_points)} nodes\033[0m"
                )
            else:
                self.get_logger().warn(
                    f"OSM 通勤路由失败: {response.message}，跳过通勤段"
                )
        except Exception as e:
            self.get_logger().error(f"OSM 通勤服务调用异常: {e}")

        # 拼接: Transit + CPP[min_idx:] (只走从最近点到末尾的半段)
        dispatch_points = transit_points + cpp_points[min_idx:]
        self._publish_path(dispatch_points)

        self.get_logger().info(
            f"\033[32m[SUCCESS] Polygon closed. "
            f"Coverage path dispatched with {len(dispatch_points)} points.\033[0m"
        )
        self.get_logger().info(
            f"[INFO] Dispatch: {len(transit_points)} transit + "
            f"{len(cpp_points) - min_idx} coverage "
            f"(slice from idx {min_idx})."
        )

    def _dispatch_without_transit(self, generation):
        """跳过通勤段，直接下发 CPP（从接入点开始）。"""
        if generation != self._request_generation:
            return
        cpp_points = self._pending_cpp_points
        min_idx = self._pending_min_idx
        if not cpp_points:
            return
        if self._wrap_coverage_from_entry and min_idx > 0:
            dispatch = cpp_points[min_idx:] + cpp_points[:min_idx]
        else:
            dispatch = cpp_points[min_idx:]
        self._publish_path(dispatch)
        self.get_logger().info(
            f"\033[32m[SUCCESS] CPP dispatched without transit: "
            f"{len(dispatch)} points from idx {min_idx}\033[0m"
        )

    # ------------------------------------------------------------------
    #  路径发布辅助
    # ------------------------------------------------------------------

    def _publish_path(self, points):
        """将 (x, y) 点列表打包为 nav_msgs/Path 并发布到 /global_plan。"""
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

    # ------------------------------------------------------------------
    #  核心算法: 最优角度弓字形扫掠 (Shapely)
    # ------------------------------------------------------------------

    def _compute_coverage_path(self, vertices):
        """基于 Shapely 的最优角度扫掠线弓字形路径生成。

        Steps:
          1. 构建 Shapely Polygon
          2. 计算最长边与 X 轴夹角 theta
          3. 将多边形绕原点旋转 -theta，使最长边水平
          4. 在旋转后的 bounding box 内每隔 sweep_spacing 生成水平扫描线
          5. 扫描线与多边形求交，按行分组获取有效线段
          6. 将线段按 S 型 (左→右, 右→左, ...) 串联
          7. 将所有点旋转 +theta 恢复真实朝向

        Returns:
            list of (x, y) tuples, or None on failure
        """
        poly = Polygon(vertices)
        if not poly.is_valid:
            poly = poly.buffer(0)
        if poly.is_empty:
            self.get_logger().error("多边形为空或无效")
            return None
        if poly.geom_type == "MultiPolygon":
            poly = max(poly.geoms, key=lambda g: g.area)
            self.get_logger().warn(
                "检测到 MultiPolygon，已自动选择最大子多边形"
            )

        coords = list(poly.exterior.coords)
        longest = 0.0
        theta = 0.0
        for i in range(len(coords) - 1):
            dx = coords[i + 1][0] - coords[i][0]
            dy = coords[i + 1][1] - coords[i][1]
            length = math.hypot(dx, dy)
            if length > longest:
                longest = length
                theta = math.atan2(dy, dx)

        self.get_logger().info(
            f"[CPP] Longest edge: {longest:.2f}m, "
            f"optimal angle: {math.degrees(theta):.1f}°"
        )

        theta_deg = math.degrees(theta)
        rotated = affinity.rotate(poly, -theta_deg, origin=(0, 0))

        minx, miny, maxx, maxy = rotated.bounds
        margin = 2.0
        spacing = self._sweep_spacing

        rows = []
        y = miny + spacing * 0.5
        while y < maxy:
            scan = LineString([(minx - margin, y), (maxx + margin, y)])
            inter = scan.intersection(rotated)

            row_segs = []
            if not inter.is_empty:
                if inter.geom_type == "LineString":
                    if inter.length > 0.01:
                        row_segs.append(inter)
                elif inter.geom_type == "MultiLineString":
                    for g in inter.geoms:
                        if g.length > 0.01:
                            row_segs.append(g)
            if row_segs:
                rows.append(row_segs)
            y += spacing

        if not rows:
            self.get_logger().error("未生成任何有效扫掠线段")
            return None

        path_points = []
        outside_connector_count = 0
        for row_idx, row_segs in enumerate(rows):
            row_segs.sort(key=lambda s: s.centroid.x)
            if row_idx % 2 == 1:
                row_segs.reverse()
            for seg in row_segs:
                pts = list(seg.coords)
                if row_idx % 2 == 1:
                    pts.reverse()
                if path_points:
                    if self._allow_outside_connectors:
                        connector = [path_points[-1], pts[0]]
                        outside_connector_count += 1
                    else:
                        connector = _safe_connector(rotated, path_points[-1], pts[0])
                        if connector is None:
                            self.get_logger().warn("跳过无法在作业区内连接的扫掠线段")
                            continue
                    path_points.extend(connector[1:])
                    path_points.extend(pts[1:])
                else:
                    path_points.extend(pts)

        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        result = []
        for x, y in path_points:
            xr = x * cos_t - y * sin_t
            yr = x * sin_t + y * cos_t
            result.append((xr, yr))

        total_segs = sum(len(r) for r in rows)
        self.get_logger().info(
            f"[CPP] Generated {total_segs} sweep segments "
            f"across {len(rows)} rows, {len(result)} total points"
        )
        if outside_connector_count:
            self.get_logger().info(
                f"[CPP] Used {outside_connector_count} direct UAV air connectors "
                "between sweep segments"
            )

        return result


def main(args=None):
    rclpy.init(args=args)
    node = PolygonCoveragePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
