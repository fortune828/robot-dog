"""OSM semantic constrained coverage planning for UAV patrol areas."""

from __future__ import annotations

import csv
import heapq
import json
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

from shapely import affinity
from shapely.geometry import GeometryCollection, LineString, MultiPolygon, Point, Polygon, box
from shapely.ops import substring, unary_union
from shapely.prepared import prep
from shapely.strtree import STRtree

from uavpatrol_navigation.geo_utils import latlon_to_local, local_to_latlon
from uavpatrol_navigation.patrol_area_io import (
    load_patrol_area,
    patrol_area_to_local_vertices,
    resolve_project_path,
)
from uavpatrol_navigation.planning_defaults import DEFAULT_PLANNING
from uavpatrol_navigation.uav_waypoints_to_dji_mission_converter import convert


@dataclass
class SemanticCoverageResult:
    origin_lat: float
    origin_lon: float
    target_area_local: Polygon
    hard_obstacles_local: Polygon | MultiPolygon | GeometryCollection
    coverage_target_area_local: Polygon | MultiPolygon | GeometryCollection
    planning_airspace_local: Polygon | MultiPolygon | GeometryCollection
    patrol_area_local: Polygon
    static_obstacles_local: list[Polygon]
    obstacle_buffers_local: list[Polygon]
    flyable_area_local: Polygon | MultiPolygon
    route_local: list[tuple[float, float]]
    route_wgs84: list[tuple[float, float]]
    transit_segments_local: list[LineString]
    diagnostics: dict | None = None
    output_dir: Path | None = None


@dataclass
class PatrolTask:
    type: str
    geometry: LineString
    priority: float
    category: str = "semantic"
    id: int = -1

    @property
    def path(self):
        return list(self.geometry.coords)

    @property
    def length(self):
        return self.geometry.length


def _parse_float_tag(tags, *keys):
    for key in keys:
        value = tags.get(key)
        if value is None:
            continue
        text = str(value).strip().lower().replace("m", "")
        try:
            return float(text)
        except ValueError:
            continue
    return None


def _building_height(tags, default_height):
    height = _parse_float_tag(tags, "height", "building:height")
    if height is not None:
        return height
    levels = _parse_float_tag(tags, "building:levels")
    if levels is not None:
        return levels * 3.0
    return default_height


def _clean_polygon(poly):
    if poly.is_empty:
        return None
    if not poly.is_valid:
        poly = poly.buffer(0)
    if poly.is_empty:
        return None
    if poly.geom_type == "Polygon":
        return poly
    if poly.geom_type == "MultiPolygon":
        return max(poly.geoms, key=lambda geom: geom.area)
    return None


def _local_polygon_from_latlon(points, origin_lat, origin_lon):
    coords = [
        latlon_to_local(lat, lon, origin_lat, origin_lon, 0.0)[:2]
        for lat, lon in points
    ]
    if len(coords) >= 2 and coords[0] == coords[-1]:
        coords = coords[:-1]
    if len(coords) < 3:
        return None
    return _clean_polygon(Polygon(coords))


def _way_latlon_points(way, nodes):
    refs = [nd.get("ref") for nd in way.findall("nd")]
    points = [nodes[ref] for ref in refs if ref in nodes]
    if len(points) >= 2 and points[0] == points[-1]:
        points = points[:-1]
    return points


def _local_line_from_latlon(points, origin_lat, origin_lon):
    coords = [
        latlon_to_local(lat, lon, origin_lat, origin_lon, 0.0)[:2]
        for lat, lon in points
    ]
    if len(coords) < 2:
        return None
    line = LineString(coords)
    return line if line.length > 0.01 else None


def extract_osm_buildings(
    map_file,
    origin_lat,
    origin_lon,
    *,
    default_building_height_m=DEFAULT_PLANNING.default_building_height_m,
    context_area_local=None,
    simplify_tolerance_m=DEFAULT_PLANNING.simplify_tolerance_m,
):
    """Extract OSM building polygons in local metric coordinates."""
    map_path = resolve_project_path(map_file)
    root = ET.parse(map_path).getroot()
    nodes = {
        node.get("id"): (float(node.get("lat")), float(node.get("lon")))
        for node in root.findall("node")
    }

    buildings = []
    for way in root.findall("way"):
        tags = {tag.get("k"): tag.get("v") for tag in way.findall("tag")}
        if "building" not in tags:
            continue
        poly = _local_polygon_from_latlon(
            _way_latlon_points(way, nodes), origin_lat, origin_lon
        )
        if poly is None:
            continue
        if simplify_tolerance_m > 0.0:
            poly = poly.simplify(simplify_tolerance_m, preserve_topology=True)
            poly = _clean_polygon(poly)
            if poly is None:
                continue
        if context_area_local is not None and not poly.intersects(context_area_local):
            continue
        buildings.append(
            {
                "geometry": poly,
                "height_m": _building_height(tags, default_building_height_m),
                "tags": tags,
            }
        )
    return buildings


def extract_osm_patrol_semantics(
    map_file,
    origin_lat,
    origin_lon,
    *,
    context_area_local=None,
    simplify_tolerance_m=DEFAULT_PLANNING.simplify_tolerance_m,
):
    """Extract lightweight OSM patrol targets in local metric coordinates."""
    map_path = resolve_project_path(map_file)
    root = ET.parse(map_path).getroot()
    nodes = {
        node.get("id"): (float(node.get("lat")), float(node.get("lon")))
        for node in root.findall("node")
    }
    semantics = {
        "roads": [],
        "footways": [],
        "water": [],
        "open_areas": [],
        "forests": [],
    }

    for way in root.findall("way"):
        tags = {tag.get("k"): tag.get("v") for tag in way.findall("tag")}
        points = _way_latlon_points(way, nodes)
        if len(points) < 2:
            continue

        highway = tags.get("highway")
        if highway or tags.get("footway") or tags.get("path"):
            task_type = "FOOTWAY_TASK" if highway in {"footway", "path", "pedestrian"} or tags.get("footway") or tags.get("path") else "ROAD_PATH_TASK"
            target_key = "footways" if task_type == "FOOTWAY_TASK" else "roads"
            line = _local_line_from_latlon(points, origin_lat, origin_lon)
            if line is not None and (
                context_area_local is None or line.intersects(context_area_local)
            ):
                clipped = line.intersection(context_area_local) if context_area_local is not None else line
                parts = _lines_from_intersection(clipped)
                semantics[target_key].extend(parts)
            continue

        poly = None
        if len(points) >= 3:
            poly = _local_polygon_from_latlon(points, origin_lat, origin_lon)
            if poly is not None and simplify_tolerance_m > 0.0:
                poly = poly.simplify(simplify_tolerance_m, preserve_topology=True)
                poly = _clean_polygon(poly)
        if poly is None:
            continue
        if context_area_local is not None:
            if not poly.intersects(context_area_local):
                continue
            clipped = poly.intersection(context_area_local)
        else:
            clipped = poly

        natural = tags.get("natural")
        landuse = tags.get("landuse")
        leisure = tags.get("leisure")
        if natural == "water" or tags.get("water") or landuse in {"reservoir", "basin"}:
            parts = list(_iter_polygons(clipped))
            semantics["water"].extend(parts)
        elif landuse in {"grass", "meadow", "recreation_ground"} or leisure in {"park", "recreation_ground"} or natural == "grassland":
            parts = list(_iter_polygons(clipped))
            semantics["open_areas"].extend(parts)
        elif landuse == "forest" or natural == "wood":
            parts = list(_iter_polygons(clipped))
            semantics["forests"].extend(parts)

    return semantics


def _iter_polygons(geom):
    if geom.is_empty:
        return
    if geom.geom_type == "Polygon":
        yield geom
    elif geom.geom_type == "MultiPolygon":
        yield from geom.geoms
    elif geom.geom_type == "GeometryCollection":
        for part in geom.geoms:
            yield from _iter_polygons(part)


def _filter_flyable(geom, min_area):
    polys = [poly for poly in _iter_polygons(geom) if poly.area >= min_area]
    if not polys:
        return GeometryCollection()
    if len(polys) == 1:
        return polys[0]
    return MultiPolygon(polys)


def _geom_area(geom):
    return sum(poly.area for poly in _iter_polygons(geom))


def build_semantic_geometries(
    target_area,
    buildings,
    *,
    building_safety_buffer_m=DEFAULT_PLANNING.building_safety_buffer_m,
    min_flyable_area_m2=DEFAULT_PLANNING.min_flyable_area_m2,
    max_off_area_distance_m=DEFAULT_PLANNING.max_off_area_distance_m,
    simplify_tolerance_m=DEFAULT_PLANNING.simplify_tolerance_m,
):
    # Rollback policy: the user polygon is the flight boundary again.  The
    # context still includes one safety-buffer width so buildings just outside
    # the drawn line can subtract their buffer from the inside edge.
    _ = max_off_area_distance_m
    context_area = target_area.buffer(building_safety_buffer_m)
    planning_shell = target_area

    building_geoms = [
        item["geometry"]
        for item in buildings
        if not item["geometry"].is_empty
    ]
    if building_geoms:
        tree = STRtree(building_geoms)
        candidate_indexes = tree.query(context_area)
    else:
        candidate_indexes = []

    obstacles = []
    buffers = []
    for idx in candidate_indexes:
        geom = building_geoms[int(idx)]
        if not geom.intersects(context_area):
            continue
        clipped = geom.intersection(context_area)
        for poly in _iter_polygons(clipped):
            obstacles.append(poly)
        buffered = geom.buffer(
            building_safety_buffer_m, resolution=4
        )
        if simplify_tolerance_m > 0.0:
            buffered = buffered.simplify(simplify_tolerance_m, preserve_topology=True)
        buffered = buffered.intersection(context_area)
        buffers.extend(_iter_polygons(buffered))

    hard_obstacles = unary_union(buffers) if buffers else GeometryCollection()
    coverage_target_area = target_area.difference(hard_obstacles)
    planning_airspace = planning_shell.difference(hard_obstacles)
    if simplify_tolerance_m > 0.0:
        coverage_target_area = coverage_target_area.simplify(
            simplify_tolerance_m,
            preserve_topology=True,
        )
        planning_airspace = planning_airspace.simplify(
            simplify_tolerance_m,
            preserve_topology=True,
        )
    coverage_target_area = _filter_flyable(
        coverage_target_area,
        min_flyable_area_m2,
    )
    planning_airspace = _filter_flyable(planning_airspace, 1.0)
    return obstacles, buffers, hard_obstacles, coverage_target_area, planning_airspace


def make_flyable_area(
    patrol_area_local,
    buildings,
    *,
    building_safety_buffer_m=DEFAULT_PLANNING.building_safety_buffer_m,
    min_flyable_area_m2=DEFAULT_PLANNING.min_flyable_area_m2,
    obstacle_context_margin_m=0.0,
):
    """Backward-compatible wrapper for older callers/tests."""
    _ = obstacle_context_margin_m
    obstacles, buffers, _, coverage_target_area, _ = build_semantic_geometries(
        patrol_area_local,
        buildings,
        building_safety_buffer_m=building_safety_buffer_m,
        min_flyable_area_m2=min_flyable_area_m2,
        max_off_area_distance_m=0.0,
    )
    return obstacles, buffers, coverage_target_area


def _geometry_longest_edge_angle(geom):
    largest = max(_iter_polygons(geom), key=lambda poly: poly.area, default=None)
    if largest is None:
        return 0.0
    coords = list(largest.exterior.coords)
    longest = 0.0
    theta = 0.0
    for p1, p2 in zip(coords, coords[1:]):
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        length = math.hypot(dx, dy)
        if length > longest:
            longest = length
            theta = math.atan2(dy, dx)
    return theta


def _lines_from_intersection(intersection):
    if intersection.is_empty:
        return []
    if intersection.geom_type == "LineString":
        return [intersection] if intersection.length > 0.01 else []
    if intersection.geom_type == "MultiLineString":
        return [line for line in intersection.geoms if line.length > 0.01]
    if intersection.geom_type == "GeometryCollection":
        lines = []
        for part in intersection.geoms:
            lines.extend(_lines_from_intersection(part))
        return lines
    return []


def _safe_connector(allowed_geom, start, end, *, max_visibility_vertices=2000):
    direct = LineString([start, end])
    if allowed_geom.covers(direct):
        return [start, end]

    vertices = [start, end]
    for poly in _iter_polygons(allowed_geom):
        simple_poly = poly.simplify(0.75, preserve_topology=True)
        vertices.extend(list(simple_poly.exterior.coords)[:-1])
        for ring in simple_poly.interiors:
            vertices.extend(list(ring.coords)[:-1])
    if len(vertices) > max_visibility_vertices:
        return None

    graph = [[] for _ in vertices]
    for i in range(len(vertices)):
        for j in range(i + 1, len(vertices)):
            segment = LineString([vertices[i], vertices[j]])
            if allowed_geom.covers(segment):
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


class _GridMask:
    neighbors = (
        (-1, -1), (0, -1), (1, -1),
        (-1, 0),           (1, 0),
        (-1, 1),  (0, 1),  (1, 1),
    )

    def __init__(self, allowed_geom, *, resolution=3.0, max_cells=180000):
        self.allowed_geom = allowed_geom
        self.resolution = resolution
        self.valid = bool(not allowed_geom.is_empty and resolution > 0.0)
        if not self.valid:
            self.width = 0
            self.height = 0
            return
        minx, miny, maxx, maxy = allowed_geom.bounds
        self.minx = minx - resolution * 2
        self.miny = miny - resolution * 2
        self.maxx = maxx + resolution * 2
        self.maxy = maxy + resolution * 2
        self.width = int(math.ceil((self.maxx - self.minx) / resolution)) + 1
        self.height = int(math.ceil((self.maxy - self.miny) / resolution)) + 1
        self.valid = self.width > 0 and self.height > 0 and self.width * self.height <= max_cells
        self.walkable_cache = {}
        self.component_cache = {}
        self.component_by_cell = {}
        self.next_component_id = 0

    def in_bounds(self, cell):
        return 0 <= cell[0] < self.width and 0 <= cell[1] < self.height

    def to_cell(self, point):
        x, y = point
        return (
            max(0, min(self.width - 1, int(round((x - self.minx) / self.resolution)))),
            max(0, min(self.height - 1, int(round((y - self.miny) / self.resolution)))),
        )

    def to_point(self, cell):
        ix, iy = cell
        return (self.minx + ix * self.resolution, self.miny + iy * self.resolution)

    def is_walkable(self, cell):
        if not self.valid or not self.in_bounds(cell):
            return False
        cached = self.walkable_cache.get(cell)
        if cached is not None:
            return cached
        value = self.allowed_geom.covers(Point(self.to_point(cell)))
        self.walkable_cache[cell] = value
        return value

    def edge_clear(self, a, b):
        return self.allowed_geom.covers(LineString([self.to_point(a), self.to_point(b)]))

    def iter_neighbors(self, cell):
        for dx, dy in self.neighbors:
            nxt = (cell[0] + dx, cell[1] + dy)
            if self.is_walkable(nxt) and self.edge_clear(cell, nxt):
                yield nxt

    def snap_cell(self, point, *, max_radius=4):
        if not self.valid:
            return None
        base = self.to_cell(point)
        for radius in range(max_radius + 1):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if max(abs(dx), abs(dy)) != radius:
                        continue
                    cell = (base[0] + dx, base[1] + dy)
                    if not self.is_walkable(cell):
                        continue
                    if self.allowed_geom.covers(LineString([point, self.to_point(cell)])):
                        return cell
        return None

    def component_id(self, cell):
        if cell is None or not self.is_walkable(cell):
            return None
        cached = self.component_by_cell.get(cell)
        if cached is not None:
            return cached
        component_id = self.next_component_id
        self.next_component_id += 1
        queue = [cell]
        self.component_by_cell[cell] = component_id
        for current in queue:
            for nxt in self.iter_neighbors(current):
                if nxt in self.component_by_cell:
                    continue
                self.component_by_cell[nxt] = component_id
                queue.append(nxt)
        return component_id

    def path(self, start, end):
        start_cell = self.snap_cell(start)
        if start_cell is None:
            return None, "start_invalid"
        end_cell = self.snap_cell(end)
        if end_cell is None:
            return None, "goal_invalid"
        if self.component_id(start_cell) != self.component_id(end_cell):
            return None, "component_mismatch"

        queue = [(0.0, start_cell)]
        distances = {start_cell: 0.0}
        previous = {}
        while queue:
            _, cell = heapq.heappop(queue)
            if cell == end_cell:
                break
            base_distance = distances[cell]
            for nxt in self.iter_neighbors(cell):
                dx = nxt[0] - cell[0]
                dy = nxt[1] - cell[1]
                step = math.hypot(dx, dy) * self.resolution
                candidate = base_distance + step
                if candidate < distances.get(nxt, float("inf")):
                    distances[nxt] = candidate
                    previous[nxt] = cell
                    hx = (nxt[0] - end_cell[0]) * self.resolution
                    hy = (nxt[1] - end_cell[1]) * self.resolution
                    heapq.heappush(queue, (candidate + math.hypot(hx, hy), nxt))

        if end_cell not in distances:
            return None, "open_set_exhausted"

        cells = [end_cell]
        while cells[-1] != start_cell:
            cells.append(previous[cells[-1]])
        cells.reverse()

        points = [start]
        points.extend(self.to_point(cell) for cell in cells)
        points.append(end)
        return _remove_redundant_points(points), "ok"


def _as_patrol_lines(geom):
    if geom is None or geom.is_empty:
        return []
    if geom.geom_type in {"LineString", "MultiLineString", "GeometryCollection"}:
        return _lines_from_intersection(geom)
    if geom.geom_type == "Polygon":
        return _lines_from_intersection(geom.boundary)
    if geom.geom_type == "MultiPolygon":
        lines = []
        for poly in geom.geoms:
            lines.extend(_lines_from_intersection(poly.boundary))
        return lines
    return []


def _open_area_sweep_tasks(
    area,
    flyable_area,
    *,
    spacing_m,
    priority,
    min_task_length_m,
    task_type="OPEN_AREA_TASK",
):
    tasks = []
    open_area = area.intersection(flyable_area)
    if open_area.is_empty:
        return tasks
    theta = _geometry_longest_edge_angle(open_area)
    theta_deg = math.degrees(theta)
    rotated = affinity.rotate(open_area, -theta_deg, origin=(0, 0))
    minx, miny, maxx, maxy = rotated.bounds
    y = miny + spacing_m * 0.5
    while y < maxy:
        scan = LineString([(minx - spacing_m, y), (maxx + spacing_m, y)])
        for line in _lines_from_intersection(scan.intersection(rotated)):
            line = affinity.rotate(line, theta_deg, origin=(0, 0))
            if line.length >= min_task_length_m:
                tasks.append(PatrolTask(task_type, line, priority, category="coverage"))
        y += spacing_m
    return tasks


def build_patrol_tasks(
    *,
    roads,
    footways,
    buildings,
    water_areas,
    open_areas,
    forest_areas,
    flyable_area,
    user_polygon,
    building_perimeter_offset_m=DEFAULT_PLANNING.building_perimeter_offset_m,
    open_area_sweep_spacing_m=DEFAULT_PLANNING.open_area_sweep_spacing_m,
    min_task_length_m=DEFAULT_PLANNING.min_patrol_task_length_m,
    coverage_sweep_task_priority=DEFAULT_PLANNING.coverage_sweep_task_priority,
    open_area_task_priority=DEFAULT_PLANNING.open_area_task_priority,
    road_task_priority=DEFAULT_PLANNING.road_task_priority,
    footway_task_priority=DEFAULT_PLANNING.footway_task_priority,
    shoreline_task_priority=DEFAULT_PLANNING.shoreline_task_priority,
    forest_edge_task_priority=DEFAULT_PLANNING.forest_edge_task_priority,
    building_perimeter_task_priority=DEFAULT_PLANNING.building_perimeter_task_priority,
):
    tasks = []

    def add_task(task_type, geom, priority, *, category="semantic"):
        if geom is None or geom.is_empty:
            return
        for line in _as_patrol_lines(geom):
            if line.length <= 0.01:
                continue
            clipped = line.intersection(flyable_area)
            clipped_parts = _as_patrol_lines(clipped)
            for part in clipped_parts:
                if part.length >= min_task_length_m:
                    tasks.append(PatrolTask(task_type, part, priority, category=category))

    tasks.extend(
        _open_area_sweep_tasks(
            flyable_area,
            flyable_area,
            spacing_m=open_area_sweep_spacing_m,
            priority=coverage_sweep_task_priority,
            min_task_length_m=min_task_length_m,
            task_type="COVERAGE_SWEEP_TASK",
        )
    )

    for road in roads:
        add_task("ROAD_PATH_TASK", road, road_task_priority)
    for footway in footways:
        add_task("FOOTWAY_TASK", footway, footway_task_priority)
    for building in buildings:
        ring = building["geometry"].buffer(building_perimeter_offset_m, resolution=4).boundary
        add_task("BUILDING_PERIMETER_TASK", ring, building_perimeter_task_priority)
    for water in water_areas:
        add_task("SHORELINE_TASK", water.boundary.intersection(user_polygon), shoreline_task_priority)
    for area in open_areas:
        tasks.extend(
            _open_area_sweep_tasks(
                area,
                flyable_area,
                spacing_m=open_area_sweep_spacing_m,
                priority=open_area_task_priority,
                min_task_length_m=min_task_length_m,
                task_type="OPEN_AREA_TASK",
            )
        )
    for forest in forest_areas:
        add_task("FOREST_EDGE_TASK", forest.boundary.intersection(user_polygon), forest_edge_task_priority)

    for idx, task in enumerate(sorted(tasks, key=lambda task: (-task.priority, -task.length))):
        task.id = idx
    return sorted(tasks, key=lambda task: (-task.priority, -task.length))


def _oriented_task_path(task, current):
    path = task.path
    if len(path) <= 1:
        return path
    forward_distance = math.hypot(path[0][0] - current[0], path[0][1] - current[1])
    backward_distance = math.hypot(path[-1][0] - current[0], path[-1][1] - current[1])
    return list(reversed(path)) if backward_distance < forward_distance else path


def _line_points(line):
    return list(line.coords) if not line.is_empty else []


def _task_anchor_path(task, current, grid_mask):
    current_cell = grid_mask.snap_cell(current)
    if current_cell is None:
        return None, "start_invalid"
    current_component = grid_mask.component_id(current_cell)
    best = None
    for line in _lines_from_intersection(task.geometry.intersection(grid_mask.allowed_geom)):
        if line.length <= 0.01:
            continue
        distances = [
            0.0,
            min(2.0, line.length),
            min(5.0, line.length),
            line.length,
            max(0.0, line.length - 2.0),
            max(0.0, line.length - 5.0),
            line.length * 0.5,
        ]
        seen = set()
        anchors = []
        for distance in distances:
            key = round(distance, 3)
            if key in seen:
                continue
            seen.add(key)
            anchor = line.interpolate(distance).coords[0]
            anchor_cell = grid_mask.snap_cell(anchor)
            if anchor_cell is None:
                continue
            if grid_mask.component_id(anchor_cell) != current_component:
                continue
            anchors.append((distance, anchor))
        if not anchors:
            continue
        entry_distance, entry_point = min(
            anchors,
            key=lambda item: math.hypot(item[1][0] - current[0], item[1][1] - current[1]),
        )
        exit_candidates = [
            item for item in anchors
            if abs(item[0] - entry_distance) >= min(1.0, line.length * 0.1)
        ]
        if not exit_candidates:
            continue
        exit_distance, exit_point = max(
            exit_candidates,
            key=lambda item: abs(item[0] - entry_distance),
        )
        segment = substring(line, entry_distance, exit_distance)
        path = _line_points(segment)
        if len(path) < 2:
            continue
        if math.hypot(path[0][0] - entry_point[0], path[0][1] - entry_point[1]) > 1e-6:
            path.insert(0, entry_point)
        if math.hypot(path[-1][0] - exit_point[0], path[-1][1] - exit_point[1]) > 1e-6:
            path.append(exit_point)
        distance_from_current = math.hypot(entry_point[0] - current[0], entry_point[1] - current[1])
        if best is None or distance_from_current < best[0]:
            best = (distance_from_current, path)
    if best is None:
        return None, "component_mismatch"
    return best[1], "ok"


def _connect_patrol_points(flyable_area, grid_mask, start, end):
    direct = LineString([start, end])
    if flyable_area.covers(direct):
        return [start, end], "direct"
    connector, reason = grid_mask.path(start, end)
    return connector, "A*" if connector is not None else reason


def plan_patrol_task_route(
    tasks,
    flyable_area,
    *,
    connector_grid_resolution_m=DEFAULT_PLANNING.connector_grid_resolution_m,
    max_patrol_tasks=DEFAULT_PLANNING.max_patrol_tasks,
    coverage_score_weight=DEFAULT_PLANNING.coverage_score_weight,
    semantic_score_weight=DEFAULT_PLANNING.semantic_score_weight,
    detour_score_penalty=DEFAULT_PLANNING.detour_score_penalty,
    building_perimeter_task_fraction=DEFAULT_PLANNING.building_perimeter_task_fraction,
    semantic_side_max_detour_m=DEFAULT_PLANNING.semantic_side_max_detour_m,
):
    selected_counts = {
        "coverage": 0,
        "semantic": 0,
        "building_perimeter": 0,
    }
    if not tasks or flyable_area.is_empty:
        return [], selected_counts
    remaining = list(tasks[:max_patrol_tasks])
    route_area = flyable_area
    grid_mask = _GridMask(
        route_area,
        resolution=connector_grid_resolution_m,
        max_cells=180000,
    )
    start_point = route_area.representative_point()
    current = (start_point.x, start_point.y)
    route = []
    max_building_perimeter_tasks = max(
        1,
        int(max_patrol_tasks * building_perimeter_task_fraction),
    )

    while remaining:
        ranked = []
        for idx, task in enumerate(remaining):
            if (
                task.type == "BUILDING_PERIMETER_TASK"
                and selected_counts["building_perimeter"] >= max_building_perimeter_tasks
            ):
                continue
            path, anchor_reason = _task_anchor_path(task, current, grid_mask)
            if path is None or len(path) < 2:
                _ = anchor_reason
                continue
            distance = math.hypot(path[0][0] - current[0], path[0][1] - current[1])
            if task.category != "coverage" and distance > semantic_side_max_detour_m:
                continue
            coverage_gain = task.length if task.category == "coverage" else 0.0
            semantic_bonus = task.length * task.priority if task.category != "coverage" else 0.0
            score = (
                coverage_gain * coverage_score_weight
                + semantic_bonus * semantic_score_weight
                - distance * detour_score_penalty
            )
            ranked.append((-score, idx, path))
        if not ranked:
            break
        ranked.sort()

        selected = None
        for _, idx, path in ranked:
            if not route:
                selected = (idx, path, None)
                break
            connector, method = _connect_patrol_points(
                route_area,
                grid_mask,
                current,
                path[0],
            )
            if connector is not None:
                _ = method
                selected = (idx, path, connector)
                break
            _ = method
        if selected is None:
            break

        idx, path, connector = selected
        if route and connector:
            route.extend(connector[1:])
        route.extend(path if not route else path[1:])
        current = route[-1]
        task = remaining.pop(idx)
        if task.category == "coverage":
            selected_counts["coverage"] += 1
        else:
            selected_counts["semantic"] += 1
            if task.type == "BUILDING_PERIMETER_TASK":
                selected_counts["building_perimeter"] += 1
    simplified = _remove_redundant_points(route)
    return simplified, selected_counts


def _fallback_boundary_patrol_route(flyable_area, *, min_length_m):
    lines = sorted(_as_patrol_lines(flyable_area.boundary), key=lambda line: line.length, reverse=True)
    for line in lines:
        if line.length < min_length_m:
            continue
        points = list(line.coords)
        if len(points) >= 2 and points[0] == points[-1]:
            points = points[:-1]
        if len(points) >= 2:
            return _remove_redundant_points(points)
    return []


def compute_semantic_coverage_path(
    coverage_target_area_local,
    *,
    planning_airspace_local=None,
    hard_obstacles_local=None,
    coverage_spacing_m=DEFAULT_PLANNING.coverage_spacing_m,
    obstacle_buffers_local=None,
    route_edge_margin_m=DEFAULT_PLANNING.route_edge_margin_m,
    max_off_area_distance_m=DEFAULT_PLANNING.max_off_area_distance_m,
    sensor_coverage_radius_m=DEFAULT_PLANNING.sensor_coverage_radius_m,
    min_coverage_contribution_m=DEFAULT_PLANNING.min_coverage_contribution_m,
    max_connection_length_m=DEFAULT_PLANNING.max_connection_length_m,
    max_2opt_iterations=DEFAULT_PLANNING.max_2opt_iterations,
    disable_2opt_if_strokes_gt=DEFAULT_PLANNING.disable_2opt_if_strokes_gt,
):
    """Generate a coverage route from target coverage geometry and airspace."""
    if coverage_target_area_local.is_empty:
        return []
    if coverage_spacing_m <= 0.0:
        raise ValueError("coverage_spacing_m must be positive")
    if max_off_area_distance_m < 0.0:
        raise ValueError("max_off_area_distance_m must be non-negative")
    if sensor_coverage_radius_m < 0.0:
        raise ValueError("sensor_coverage_radius_m must be non-negative")
    if min_coverage_contribution_m < 0.0:
        raise ValueError("min_coverage_contribution_m must be non-negative")
    if max_connection_length_m <= 0.0:
        raise ValueError("max_connection_length_m must be positive")

    if hard_obstacles_local is None:
        hard_obstacles_local = unary_union(obstacle_buffers_local or [])
    if planning_airspace_local is None:
        planning_airspace_local = coverage_target_area_local.difference(hard_obstacles_local)
        planning_airspace_local = _filter_flyable(planning_airspace_local, 1.0)
    if planning_airspace_local.is_empty:
        return []

    coverage_area = _centered_coverage_area(
        coverage_target_area_local,
        route_edge_margin_m=route_edge_margin_m,
        min_retained_area_ratio=0.12,
    )
    boundary_observation_area = _boundary_observation_area(
        coverage_target_area_local,
        route_edge_margin_m=route_edge_margin_m,
    )
    coverage_sensor_area = coverage_target_area_local
    if sensor_coverage_radius_m > 0.0:
        coverage_sensor_area = coverage_target_area_local.buffer(sensor_coverage_radius_m)

    theta = _geometry_longest_edge_angle(coverage_area)
    theta_deg = math.degrees(theta)
    rotated_sensor = affinity.rotate(coverage_sensor_area, -theta_deg, origin=(0, 0))
    rotated_airspace = affinity.rotate(planning_airspace_local, -theta_deg, origin=(0, 0))
    minx, miny, maxx, maxy = rotated_airspace.bounds

    def build_strokes(spacing):
        margin = spacing
        strokes_out = []
        y = miny + spacing * 0.5
        while y < maxy:
            scan = LineString([(minx - margin, y), (maxx + margin, y)])
            for line in _lines_from_intersection(scan.intersection(rotated_airspace)):
                coverage_part = line.intersection(rotated_sensor)
                if coverage_part.length < min_coverage_contribution_m:
                    continue
                pts = _sample_line_points(line, edge_margin=0.0)
                if pts:
                    strokes_out.append(pts)
            y += spacing
        return strokes_out

    spacing = coverage_spacing_m
    strokes = build_strokes(spacing)
    while len(strokes) < 3 and spacing > 2.0:
        spacing = max(2.0, spacing * 0.5)
        strokes = build_strokes(spacing)

    obstacle_union = hard_obstacles_local
    obstacle_prepared = None if obstacle_union.is_empty else prep(obstacle_union)
    transit_area = planning_airspace_local
    rotated_boundary = affinity.rotate(boundary_observation_area, -theta_deg, origin=(0, 0))
    rotated_obstacles = affinity.rotate(obstacle_union, -theta_deg, origin=(0, 0))
    rotated_obstacle_prepared = None if rotated_obstacles.is_empty else prep(rotated_obstacles)
    strokes.extend(_boundary_strokes_from_area(rotated_boundary, coverage_spacing_m))
    route_rotated = _order_strokes_nearest_safe(
        strokes,
        rotated_airspace,
        rotated_obstacles,
        coverage_spacing_m=coverage_spacing_m,
        max_connection_length_m=max_connection_length_m,
        obstacle_prepared=rotated_obstacle_prepared,
    )

    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    route = []
    for x, y in route_rotated:
        xr = x * cos_t - y * sin_t
        yr = x * sin_t + y * cos_t
        point = (xr, yr)
        if obstacle_union.is_empty or not obstacle_union.covers(Point(point)):
            route.append(point)
    route = _sanitize_route_segments(
        route,
        transit_area,
        obstacle_union,
        obstacle_prepared,
        resolution=max(2.0, min(coverage_spacing_m, 4.0)),
    )
    if len(strokes) <= disable_2opt_if_strokes_gt:
        route = _reduce_route_crossings(
            route,
            transit_area,
            obstacle_union,
            obstacle_prepared,
            max_iterations=max_2opt_iterations,
        )
    return _remove_redundant_points(route)


def _route_connector(
    transit_area,
    obstacle_union,
    obstacle_prepared,
    start,
    end,
    *,
    coverage_spacing_m,
    max_connection_length_m,
    max_candidate_strokes=40,
):
    direct = LineString([start, end])
    if direct.length > max_connection_length_m:
        return None
    obstacle_clear = not _obstacle_intersects(direct, obstacle_union, obstacle_prepared)
    if obstacle_clear and transit_area.covers(direct):
        return [start, end]
    return None


def _connector_search_areas(allowed_geom, start, end, *, base_padding):
    seen_bounds = set()
    for padding in (base_padding, base_padding * 2.0, base_padding * 4.0):
        minx = min(start[0], end[0]) - padding
        miny = min(start[1], end[1]) - padding
        maxx = max(start[0], end[0]) + padding
        maxy = max(start[1], end[1]) + padding
        window = box(minx, miny, maxx, maxy)
        search_area = allowed_geom.intersection(window)
        if search_area.is_empty:
            continue
        bounds = tuple(round(value, 3) for value in search_area.bounds)
        if bounds in seen_bounds:
            continue
        seen_bounds.add(bounds)
        yield search_area


def _coverage_strokes_from_rows(rows, *, edge_margin):
    strokes = []
    for row in rows:
        for line in sorted(row, key=lambda item: (item.centroid.y, item.centroid.x)):
            pts = _sample_line_points(line, edge_margin=edge_margin)
            if pts:
                strokes.append(pts)
    return strokes


def _boundary_strokes_from_area(coverage_area, coverage_spacing_m):
    if coverage_area.is_empty:
        return []
    simplify_tolerance = max(0.75, min(coverage_spacing_m * 0.25, 3.0))
    min_edge_length = max(1.0, coverage_spacing_m * 0.2)
    strokes = []
    for poly in _iter_polygons(coverage_area):
        if poly.area < 1.0:
            continue
        simple_poly = poly.simplify(simplify_tolerance, preserve_topology=True)
        for ring in [simple_poly.exterior, *simple_poly.interiors]:
            points = list(ring.coords)
            if len(points) >= 2 and points[0] == points[-1]:
                points = points[:-1]
            if len(points) < 2:
                continue
            for start, end in zip(points, points[1:] + points[:1]):
                if math.hypot(end[0] - start[0], end[1] - start[1]) >= min_edge_length:
                    strokes.append([start, end])
    return strokes


def _order_strokes_nearest_safe(
    strokes,
    transit_area,
    obstacle_union,
    *,
    coverage_spacing_m,
    max_connection_length_m,
    obstacle_prepared=None,
    max_candidate_strokes=40,
):
    if not strokes:
        return []

    remaining = [
        {
            "points": list(stroke),
            "centroid": (
                sum(point[0] for point in stroke) / len(stroke),
                sum(point[1] for point in stroke) / len(stroke),
            ),
        }
        for stroke in strokes
    ]
    first_idx = min(
        range(len(remaining)),
        key=lambda idx: (remaining[idx]["centroid"][1], remaining[idx]["centroid"][0]),
    )
    first = remaining.pop(first_idx)["points"]
    if len(first) >= 2 and first[0][0] > first[-1][0]:
        first = list(reversed(first))
    route = list(first)

    while remaining:
        selected = None
        best_crossing = None
        current = route[-1]
        candidate_order = sorted(
            range(len(remaining)),
            key=lambda idx: _stroke_distance(current, remaining[idx]["points"]),
        )[:max_candidate_strokes]
        for idx in candidate_order:
            for oriented in _stroke_orientations(remaining[idx]["points"], current):
                connector = _route_connector(
                    transit_area,
                    obstacle_union,
                    obstacle_prepared,
                    current,
                    oriented[0],
                    coverage_spacing_m=coverage_spacing_m,
                    max_connection_length_m=max_connection_length_m,
                )
                if connector is None:
                    continue
                candidate_path = connector + oriented[1:]
                score = _path_length(candidate_path)
                crossing = _path_crosses_existing(candidate_path, route)
                candidate = (score, idx, oriented, connector)
                if not crossing:
                    selected = candidate
                    break
                if best_crossing is None or score < best_crossing[0]:
                    best_crossing = candidate
            if selected is not None:
                break
        if selected is None:
            selected = best_crossing
        if selected is None:
            break
        _, idx, oriented, connector = selected
        route.extend(connector[1:])
        route.extend(oriented[1:])
        remaining.pop(idx)
    return route


def _stroke_orientations(stroke, current):
    if len(stroke) <= 1:
        return [stroke]
    forward = list(stroke)
    backward = list(reversed(stroke))
    return sorted(
        [forward, backward],
        key=lambda item: math.hypot(item[0][0] - current[0], item[0][1] - current[1]),
    )


def _stroke_distance(point, stroke):
    return min(math.hypot(endpoint[0] - point[0], endpoint[1] - point[1]) for endpoint in (stroke[0], stroke[-1]))


def _path_length(points):
    return sum(
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(points, points[1:])
    )


def _path_crosses_existing(candidate_path, existing_route):
    if len(candidate_path) < 2 or len(existing_route) < 2:
        return False
    existing = LineString(existing_route)
    for a, b in zip(candidate_path, candidate_path[1:]):
        segment = LineString([a, b])
        if segment.length > 1e-6 and segment.crosses(existing):
            return True
    return False


def _obstacle_intersects(segment, obstacle_union, obstacle_prepared=None):
    if obstacle_union is None or obstacle_union.is_empty:
        return False
    if obstacle_prepared is not None:
        return obstacle_prepared.intersects(segment)
    return obstacle_union.intersects(segment)


def _reduce_route_crossings(points, transit_area, obstacle_union, obstacle_prepared=None, *, max_iterations=200):
    if len(points) < 4:
        return points
    route = list(points)

    def safe_segment(a, b):
        segment = LineString([a, b])
        if not transit_area.covers(segment):
            return False
        if _obstacle_intersects(segment, obstacle_union, obstacle_prepared):
            return False
        return True

    iterations = 0
    while iterations < max_iterations:
        changed = False
        segments = [LineString([a, b]) for a, b in zip(route, route[1:])]
        for i in range(len(segments) - 2):
            for j in range(i + 2, len(segments)):
                if j == i + 1:
                    continue
                if not segments[i].crosses(segments[j]):
                    continue
                a = route[i]
                b = route[i + 1]
                c = route[j]
                d = route[j + 1]
                if not safe_segment(a, c) or not safe_segment(b, d):
                    continue
                candidate = route[: i + 1] + list(reversed(route[i + 1 : j + 1])) + route[j + 1 :]
                route = candidate
                changed = True
                iterations += 1
                break
            if changed:
                break
        if not changed:
            break
    return route


def _centered_coverage_area(
    flyable_area,
    *,
    route_edge_margin_m,
    min_retained_area_ratio,
):
    if route_edge_margin_m <= 0.0:
        return flyable_area
    original_area = _geom_area(flyable_area)
    if original_area <= 0.0:
        return flyable_area
    inset = flyable_area.buffer(-route_edge_margin_m)
    inset = _filter_flyable(inset, 1.0)
    if inset.is_empty:
        return flyable_area
    if _geom_area(inset) < original_area * min_retained_area_ratio:
        return flyable_area
    return inset


def _boundary_observation_area(
    flyable_area,
    *,
    route_edge_margin_m,
):
    original_area = _geom_area(flyable_area)
    if original_area <= 0.0:
        return flyable_area
    clearance = min(1.0, max(0.25, route_edge_margin_m * 0.25))
    inset = flyable_area.buffer(-clearance)
    inset = _filter_flyable(inset, 1.0)
    if inset.is_empty:
        return flyable_area
    if _geom_area(inset) < original_area * 0.05:
        return flyable_area
    return inset


def _sample_line_points(line, *, edge_margin):
    length = line.length
    if length <= 0.01:
        return []
    if length <= edge_margin * 2.0 + 0.5:
        return [line.interpolate(0.5, normalized=True).coords[0]]
    start = edge_margin
    end = length - edge_margin
    if end <= start:
        return [line.interpolate(0.5, normalized=True).coords[0]]
    return [line.interpolate(start).coords[0], line.interpolate(end).coords[0]]


def _shortcut_path(points, allowed_geom, obstacle_union):
    if len(points) <= 2:
        return points

    def can_connect(a, b):
        segment = LineString([a, b])
        if not allowed_geom.buffer(1e-6).covers(segment):
            return False
        if obstacle_union is not None and not obstacle_union.is_empty and segment.intersects(obstacle_union):
            return False
        return True

    simplified = [points[0]]
    i = 0
    while i < len(points) - 1:
        best = i + 1
        for j in range(len(points) - 1, i, -1):
            if can_connect(points[i], points[j]):
                best = j
                break
        simplified.append(points[best])
        i = best
    return simplified


def _remove_redundant_points(points, *, min_distance=0.35, angle_epsilon=0.03):
    if len(points) <= 2:
        return points
    compact = [points[0]]
    for point in points[1:]:
        if math.hypot(point[0] - compact[-1][0], point[1] - compact[-1][1]) >= min_distance:
            compact.append(point)
    if len(compact) <= 2:
        return compact
    out = [compact[0]]
    for prev, cur, nxt in zip(compact, compact[1:], compact[2:]):
        v1 = (cur[0] - prev[0], cur[1] - prev[1])
        v2 = (nxt[0] - cur[0], nxt[1] - cur[1])
        len1 = math.hypot(*v1)
        len2 = math.hypot(*v2)
        if len1 < 1e-6 or len2 < 1e-6:
            continue
        cross = abs(v1[0] * v2[1] - v1[1] * v2[0]) / (len1 * len2)
        dot = (v1[0] * v2[0] + v1[1] * v2[1]) / (len1 * len2)
        if cross < angle_epsilon and dot > 0.0:
            continue
        out.append(cur)
    out.append(compact[-1])
    return out


def _sanitize_route_segments(points, transit_area, obstacle_union, obstacle_prepared=None, *, resolution):
    if len(points) < 2 or obstacle_union.is_empty:
        return points
    safe = [points[0]]
    for point in points[1:]:
        segment = LineString([safe[-1], point])
        if not _obstacle_intersects(segment, obstacle_union, obstacle_prepared):
            safe.append(point)
    return safe


def _densify_points(points, *, max_spacing):
    if len(points) < 2 or max_spacing <= 0.0:
        return points
    dense = [points[0]]
    for start, end in zip(points, points[1:]):
        sx, sy = start
        ex, ey = end
        dist = math.hypot(ex - sx, ey - sy)
        steps = max(1, int(math.ceil(dist / max_spacing)))
        for step in range(1, steps + 1):
            t = step / steps
            dense.append((sx + (ex - sx) * t, sy + (ey - sy) * t))
    return dense


def _transform_coords_local_to_wgs84(coords, origin_lat, origin_lon):
    out = []
    for x, y in coords:
        lat, lon = local_to_latlon(x, y, origin_lat, origin_lon)
        out.append((lon, lat))
    return out


def _polygon_to_geojson_coords(poly, origin_lat, origin_lon):
    rings = [_transform_coords_local_to_wgs84(poly.exterior.coords, origin_lat, origin_lon)]
    for interior in poly.interiors:
        rings.append(_transform_coords_local_to_wgs84(interior.coords, origin_lat, origin_lon))
    return rings


def geometry_to_geojson_geometry(geom, origin_lat, origin_lon):
    if geom.is_empty:
        return {"type": "GeometryCollection", "geometries": []}
    if geom.geom_type == "LineString":
        return {
            "type": "LineString",
            "coordinates": _transform_coords_local_to_wgs84(geom.coords, origin_lat, origin_lon),
        }
    if geom.geom_type == "MultiLineString":
        return {
            "type": "MultiLineString",
            "coordinates": [
                _transform_coords_local_to_wgs84(line.coords, origin_lat, origin_lon)
                for line in geom.geoms
            ],
        }
    if geom.geom_type == "GeometryCollection":
        return {
            "type": "GeometryCollection",
            "geometries": [
                geometry_to_geojson_geometry(part, origin_lat, origin_lon)
                for part in geom.geoms
                if not part.is_empty
            ],
        }
    polys = list(_iter_polygons(geom))
    if not polys:
        return {"type": "GeometryCollection", "geometries": []}
    if len(polys) == 1:
        return {
            "type": "Polygon",
            "coordinates": _polygon_to_geojson_coords(polys[0], origin_lat, origin_lon),
        }
    return {
        "type": "MultiPolygon",
        "coordinates": [
            _polygon_to_geojson_coords(poly, origin_lat, origin_lon)
            for poly in polys
        ],
    }


def write_geojson(path, geometries, origin_lat, origin_lon, *, properties=None):
    path = resolve_project_path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not isinstance(geometries, list):
        geometries = [geometries]
    features = []
    for idx, geom in enumerate(geometries):
        if geom.is_empty:
            continue
        props = {"index": idx}
        if properties:
            props.update(properties(idx, geom) if callable(properties) else properties)
        features.append(
            {
                "type": "Feature",
                "properties": props,
                "geometry": geometry_to_geojson_geometry(geom, origin_lat, origin_lon),
            }
        )
    payload = {"type": "FeatureCollection", "features": features}
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return path


def _transit_segments_outside_target(route_points, target_area):
    segments = []
    if len(route_points) < 2:
        return segments
    for start, end in zip(route_points, route_points[1:]):
        segment = LineString([start, end])
        outside = segment.difference(target_area)
        if outside.is_empty or outside.length < 0.01:
            continue
        if outside.geom_type == "LineString":
            segments.append(outside)
        elif outside.geom_type == "MultiLineString":
            segments.extend(line for line in outside.geoms if line.length >= 0.01)
        elif outside.geom_type == "GeometryCollection":
            for part in outside.geoms:
                if part.geom_type == "LineString" and part.length >= 0.01:
                    segments.append(part)
                elif part.geom_type == "MultiLineString":
                    segments.extend(line for line in part.geoms if line.length >= 0.01)
    return segments


def _coverage_ratio(route_line, area, *, buffer_radius):
    if route_line.is_empty or area is None or area.is_empty or _geom_area(area) <= 0.0:
        return 0.0
    covered = route_line.buffer(buffer_radius).intersection(area)
    return min(1.0, _geom_area(covered) / _geom_area(area))


def _coverage_first_diagnostics(
    route_points,
    planning_airspace,
    open_areas,
    buildings,
    *,
    coverage_spacing_m,
    selected_counts,
):
    route_line = LineString(route_points) if len(route_points) >= 2 else GeometryCollection()
    open_area = unary_union(open_areas).intersection(planning_airspace) if open_areas else GeometryCollection()
    building_union = unary_union([item["geometry"] for item in buildings]) if buildings else GeometryCollection()
    point_distances = []
    if not building_union.is_empty:
        point_distances = [Point(point).distance(building_union) for point in route_points]
    buffer_radius = max(coverage_spacing_m * 0.5, 1.0)
    return {
        "selected_coverage_task_count": selected_counts.get("coverage", 0),
        "selected_semantic_task_count": selected_counts.get("semantic", 0),
        "building_perimeter_selected_count": selected_counts.get("building_perimeter", 0),
        "open_area_coverage_ratio": _coverage_ratio(
            route_line,
            open_area,
            buffer_radius=buffer_radius,
        ),
        "route_area_coverage_ratio": _coverage_ratio(
            route_line,
            planning_airspace,
            buffer_radius=buffer_radius,
        ),
        "average_distance_to_building": (
            sum(point_distances) / len(point_distances)
            if point_distances
            else None
        ),
    }


def _area_origin(area_data):
    points = area_data.get("boundary", [])
    if not points:
        raise ValueError("patrol_area.json has empty boundary")
    lat = sum(float(point["latitude"]) for point in points) / len(points)
    lon = sum(float(point["longitude"]) for point in points) / len(points)
    return lat, lon


def plan_semantic_coverage(
    *,
    map_file=DEFAULT_PLANNING.map_file,
    patrol_area_file=DEFAULT_PLANNING.patrol_area_file,
    output_dir=DEFAULT_PLANNING.route_output_dir,
    origin_lat=None,
    origin_lon=None,
    building_safety_buffer_m=DEFAULT_PLANNING.building_safety_buffer_m,
    default_building_height_m=DEFAULT_PLANNING.default_building_height_m,
    min_flyable_area_m2=DEFAULT_PLANNING.min_flyable_area_m2,
    coverage_spacing_m=DEFAULT_PLANNING.coverage_spacing_m,
    route_edge_margin_m=DEFAULT_PLANNING.route_edge_margin_m,
    max_off_area_distance_m=DEFAULT_PLANNING.max_off_area_distance_m,
    sensor_coverage_radius_m=DEFAULT_PLANNING.sensor_coverage_radius_m,
    min_coverage_contribution_m=DEFAULT_PLANNING.min_coverage_contribution_m,
    max_connection_length_m=DEFAULT_PLANNING.max_connection_length_m,
    simplify_tolerance_m=DEFAULT_PLANNING.simplify_tolerance_m,
    max_2opt_iterations=DEFAULT_PLANNING.max_2opt_iterations,
    disable_2opt_if_strokes_gt=DEFAULT_PLANNING.disable_2opt_if_strokes_gt,
    min_patrol_task_length_m=DEFAULT_PLANNING.min_patrol_task_length_m,
    building_perimeter_offset_m=DEFAULT_PLANNING.building_perimeter_offset_m,
    open_area_sweep_spacing_m=DEFAULT_PLANNING.open_area_sweep_spacing_m,
    connector_grid_resolution_m=DEFAULT_PLANNING.connector_grid_resolution_m,
    max_patrol_tasks=DEFAULT_PLANNING.max_patrol_tasks,
    coverage_score_weight=DEFAULT_PLANNING.coverage_score_weight,
    semantic_score_weight=DEFAULT_PLANNING.semantic_score_weight,
    detour_score_penalty=DEFAULT_PLANNING.detour_score_penalty,
    building_perimeter_task_fraction=DEFAULT_PLANNING.building_perimeter_task_fraction,
    semantic_side_max_detour_m=DEFAULT_PLANNING.semantic_side_max_detour_m,
    write_outputs=True,
):
    area_data, _, _ = load_patrol_area(patrol_area_file)
    if origin_lat is None or origin_lon is None:
        origin_lat, origin_lon = _area_origin(area_data)

    vertices = patrol_area_to_local_vertices(area_data, origin_lat, origin_lon)
    patrol_area = _clean_polygon(Polygon(vertices))
    if patrol_area is None:
        raise ValueError("patrol area is invalid or empty")

    building_context = patrol_area.buffer(building_safety_buffer_m)
    buildings = extract_osm_buildings(
        map_file,
        origin_lat,
        origin_lon,
        default_building_height_m=default_building_height_m,
        context_area_local=building_context,
        simplify_tolerance_m=simplify_tolerance_m,
    )
    semantics = extract_osm_patrol_semantics(
        map_file,
        origin_lat,
        origin_lon,
        context_area_local=patrol_area,
        simplify_tolerance_m=simplify_tolerance_m,
    )
    (
        obstacles,
        buffers,
        hard_obstacles,
        coverage_target_area,
        planning_airspace,
    ) = build_semantic_geometries(
        patrol_area,
        buildings,
        building_safety_buffer_m=building_safety_buffer_m,
        min_flyable_area_m2=min_flyable_area_m2,
        max_off_area_distance_m=max_off_area_distance_m,
        simplify_tolerance_m=simplify_tolerance_m,
    )
    patrol_tasks = build_patrol_tasks(
        roads=semantics["roads"],
        footways=semantics["footways"],
        buildings=buildings,
        water_areas=semantics["water"],
        open_areas=semantics["open_areas"],
        forest_areas=semantics["forests"],
        flyable_area=planning_airspace,
        user_polygon=patrol_area,
        building_perimeter_offset_m=building_perimeter_offset_m,
        open_area_sweep_spacing_m=open_area_sweep_spacing_m,
        min_task_length_m=min_patrol_task_length_m,
    )
    route_local, selected_counts = plan_patrol_task_route(
        patrol_tasks,
        planning_airspace,
        connector_grid_resolution_m=connector_grid_resolution_m,
        max_patrol_tasks=max_patrol_tasks,
        coverage_score_weight=coverage_score_weight,
        semantic_score_weight=semantic_score_weight,
        detour_score_penalty=detour_score_penalty,
        building_perimeter_task_fraction=building_perimeter_task_fraction,
        semantic_side_max_detour_m=semantic_side_max_detour_m,
    )
    if not route_local:
        route_local = _fallback_boundary_patrol_route(
            planning_airspace,
            min_length_m=min_patrol_task_length_m,
        )
        selected_counts = {
            "coverage": 0,
            "semantic": 0,
            "building_perimeter": 0,
        }
    route_wgs84 = [
        local_to_latlon(x, y, origin_lat, origin_lon)
        for x, y in route_local
    ]
    diagnostics = _coverage_first_diagnostics(
        route_local,
        planning_airspace,
        semantics["open_areas"],
        buildings,
        coverage_spacing_m=open_area_sweep_spacing_m,
        selected_counts=selected_counts,
    )

    result = SemanticCoverageResult(
        origin_lat=origin_lat,
        origin_lon=origin_lon,
        target_area_local=patrol_area,
        hard_obstacles_local=hard_obstacles,
        coverage_target_area_local=coverage_target_area,
        planning_airspace_local=planning_airspace,
        patrol_area_local=patrol_area,
        static_obstacles_local=obstacles,
        obstacle_buffers_local=buffers,
        flyable_area_local=coverage_target_area,
        route_local=route_local,
        route_wgs84=route_wgs84,
        transit_segments_local=_transit_segments_outside_target(route_local, patrol_area),
        diagnostics=diagnostics,
        output_dir=resolve_project_path(output_dir) if output_dir else None,
    )
    if write_outputs and output_dir:
        write_semantic_geojson_outputs(result, output_dir)
    return result


def write_semantic_geojson_outputs(result, output_dir="data/output"):
    output = resolve_project_path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    write_geojson(
        output / "target_area.geojson",
        result.target_area_local,
        result.origin_lat,
        result.origin_lon,
    )
    write_geojson(
        output / "coverage_target_area.geojson",
        result.coverage_target_area_local,
        result.origin_lat,
        result.origin_lon,
    )
    write_geojson(
        output / "planning_airspace.geojson",
        result.planning_airspace_local,
        result.origin_lat,
        result.origin_lon,
    )
    write_geojson(
        output / "static_obstacles.geojson",
        result.static_obstacles_local,
        result.origin_lat,
        result.origin_lon,
    )
    write_geojson(
        output / "static_obstacle_buffers.geojson",
        result.obstacle_buffers_local,
        result.origin_lat,
        result.origin_lon,
    )
    write_geojson(
        output / "flyable_area.geojson",
        result.coverage_target_area_local,
        result.origin_lat,
        result.origin_lon,
    )
    write_geojson(
        output / "transit_segments.geojson",
        result.transit_segments_local,
        result.origin_lat,
        result.origin_lon,
    )
    if result.diagnostics is not None:
        (output / "patrol_diagnostics.json").write_text(
            json.dumps(result.diagnostics, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )


def build_waypoint_payload(
    result,
    *,
    mission_name="uav_area_mission",
    altitude_m=DEFAULT_PLANNING.altitude_m,
    altitude_mode=DEFAULT_PLANNING.altitude_mode,
    speed_mps=DEFAULT_PLANNING.speed_mps,
    coordinate_frame="WGS84",
    aircraft_model="DJI Matrice 4E",
    protocol="DJI_WAYPOINT_3_0",
):
    waypoints = []
    for idx, (lat, lon) in enumerate(result.route_wgs84):
        waypoints.append(
            {
                "index": idx,
                "latitude": lat,
                "longitude": lon,
                "altitude_m": altitude_m,
                "altitude_mode": altitude_mode,
                "speed_mps": speed_mps,
                "coordinate_frame": coordinate_frame,
            }
        )
    return {
        "mission_name": mission_name,
        "protocol": protocol,
        "aircraft_model": aircraft_model,
        "coordinate_frame": coordinate_frame,
        "altitude_mode": altitude_mode,
        "origin": {
            "mode": "semantic_area_planner",
            "latitude": result.origin_lat,
            "longitude": result.origin_lon,
        },
        "source": "semantic_coverage_planner",
        "waypoint_count": len(waypoints),
        "waypoints": waypoints,
    }


def write_waypoint_files(payload, output_dir=DEFAULT_PLANNING.route_output_dir, *, prefix="uav_waypoints"):
    output = resolve_project_path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    json_path = output / f"{prefix}.json"
    csv_path = output / f"{prefix}.csv"
    json_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "index",
                "latitude",
                "longitude",
                "altitude_m",
                "altitude_mode",
                "speed_mps",
                "coordinate_frame",
            ],
        )
        writer.writeheader()
        writer.writerows(payload["waypoints"])
    return json_path, csv_path


def run_semantic_planning_to_files(
    *,
    map_file=DEFAULT_PLANNING.map_file,
    patrol_area_file=DEFAULT_PLANNING.patrol_area_file,
    output_dir=DEFAULT_PLANNING.route_output_dir,
    building_safety_buffer_m=DEFAULT_PLANNING.building_safety_buffer_m,
    default_building_height_m=DEFAULT_PLANNING.default_building_height_m,
    min_flyable_area_m2=DEFAULT_PLANNING.min_flyable_area_m2,
    coverage_spacing_m=DEFAULT_PLANNING.coverage_spacing_m,
    route_edge_margin_m=DEFAULT_PLANNING.route_edge_margin_m,
    max_off_area_distance_m=DEFAULT_PLANNING.max_off_area_distance_m,
    sensor_coverage_radius_m=DEFAULT_PLANNING.sensor_coverage_radius_m,
    min_coverage_contribution_m=DEFAULT_PLANNING.min_coverage_contribution_m,
    max_connection_length_m=DEFAULT_PLANNING.max_connection_length_m,
    simplify_tolerance_m=DEFAULT_PLANNING.simplify_tolerance_m,
    max_2opt_iterations=DEFAULT_PLANNING.max_2opt_iterations,
    disable_2opt_if_strokes_gt=DEFAULT_PLANNING.disable_2opt_if_strokes_gt,
    min_patrol_task_length_m=DEFAULT_PLANNING.min_patrol_task_length_m,
    building_perimeter_offset_m=DEFAULT_PLANNING.building_perimeter_offset_m,
    open_area_sweep_spacing_m=DEFAULT_PLANNING.open_area_sweep_spacing_m,
    connector_grid_resolution_m=DEFAULT_PLANNING.connector_grid_resolution_m,
    max_patrol_tasks=DEFAULT_PLANNING.max_patrol_tasks,
    coverage_score_weight=DEFAULT_PLANNING.coverage_score_weight,
    semantic_score_weight=DEFAULT_PLANNING.semantic_score_weight,
    detour_score_penalty=DEFAULT_PLANNING.detour_score_penalty,
    building_perimeter_task_fraction=DEFAULT_PLANNING.building_perimeter_task_fraction,
    semantic_side_max_detour_m=DEFAULT_PLANNING.semantic_side_max_detour_m,
    altitude_m=DEFAULT_PLANNING.altitude_m,
    speed_mps=DEFAULT_PLANNING.speed_mps,
    write_semantic_copy=False,
):
    result = plan_semantic_coverage(
        map_file=map_file,
        patrol_area_file=patrol_area_file,
        output_dir=output_dir,
        building_safety_buffer_m=building_safety_buffer_m,
        default_building_height_m=default_building_height_m,
        min_flyable_area_m2=min_flyable_area_m2,
        coverage_spacing_m=coverage_spacing_m,
        route_edge_margin_m=route_edge_margin_m,
        max_off_area_distance_m=max_off_area_distance_m,
        sensor_coverage_radius_m=sensor_coverage_radius_m,
        min_coverage_contribution_m=min_coverage_contribution_m,
        max_connection_length_m=max_connection_length_m,
        simplify_tolerance_m=simplify_tolerance_m,
        max_2opt_iterations=max_2opt_iterations,
        disable_2opt_if_strokes_gt=disable_2opt_if_strokes_gt,
        min_patrol_task_length_m=min_patrol_task_length_m,
        building_perimeter_offset_m=building_perimeter_offset_m,
        open_area_sweep_spacing_m=open_area_sweep_spacing_m,
        connector_grid_resolution_m=connector_grid_resolution_m,
        max_patrol_tasks=max_patrol_tasks,
        coverage_score_weight=coverage_score_weight,
        semantic_score_weight=semantic_score_weight,
        detour_score_penalty=detour_score_penalty,
        building_perimeter_task_fraction=building_perimeter_task_fraction,
        semantic_side_max_detour_m=semantic_side_max_detour_m,
        write_outputs=True,
    )
    payload = build_waypoint_payload(
        result,
        altitude_m=altitude_m,
        speed_mps=speed_mps,
    )
    json_path, csv_path = write_waypoint_files(payload, output_dir, prefix="uav_waypoints")
    if write_semantic_copy:
        write_waypoint_files(payload, output_dir, prefix="semantic_uav_waypoints")
    convert(json_path, resolve_project_path(output_dir))
    return result, json_path, csv_path
