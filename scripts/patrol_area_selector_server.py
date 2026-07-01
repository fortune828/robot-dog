#!/usr/bin/env python3
"""Local web tool for drawing and saving UAV patrol areas."""

import argparse
import json
import sys
import urllib.parse
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PKG_SRC = ROOT / "src" / "uavpatrol_navigation"
if str(PKG_SRC) not in sys.path:
    sys.path.insert(0, str(PKG_SRC))

from uavpatrol_navigation.patrol_visualization import create_visualization
from uavpatrol_navigation.patrol_area_io import resolve_project_path
from uavpatrol_navigation.planning_defaults import DEFAULT_PLANNING
from uavpatrol_navigation.semantic_coverage_planner import (
    run_semantic_planning_to_files,
)


WEB_DIR = ROOT / "web"
DEFAULT_HTML = WEB_DIR / "patrol_area_selector.html"


def read_osm_bounds(map_file):
    path = resolve_project_path(map_file)
    root = ET.parse(path).getroot()
    bounds = root.find("bounds")
    if bounds is None:
        lats = []
        lons = []
        for node in root.findall("node"):
            lats.append(float(node.get("lat")))
            lons.append(float(node.get("lon")))
        if not lats or not lons:
            raise ValueError(f"{path} does not contain bounds or nodes")
        minlat, maxlat = min(lats), max(lats)
        minlon, maxlon = min(lons), max(lons)
    else:
        minlat = float(bounds.get("minlat"))
        maxlat = float(bounds.get("maxlat"))
        minlon = float(bounds.get("minlon"))
        maxlon = float(bounds.get("maxlon"))
    return {
        "min_latitude": minlat,
        "min_longitude": minlon,
        "max_latitude": maxlat,
        "max_longitude": maxlon,
        "center": {
            "latitude": (minlat + maxlat) / 2.0,
            "longitude": (minlon + maxlon) / 2.0,
        },
    }


def patrol_area_payload(area_name, points):
    if len(points) < 3:
        raise ValueError("polygon must contain at least 3 points")
    boundary = []
    for point in points:
        lat = float(point.get("lat", point.get("latitude")))
        lng = float(point.get("lng", point.get("longitude")))
        boundary.append({"latitude": lat, "longitude": lng})
    return {
        "area_name": area_name or "custom_patrol_area",
        "coordinate_frame": "WGS84",
        "boundary": boundary,
    }


def write_json(path_value, payload):
    path = resolve_project_path(path_value)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return path


def invalidate_route_outputs(output_files):
    removed = []
    for path_value in output_files:
        path = resolve_project_path(path_value)
        if path.exists():
            path.unlink()
            removed.append(str(path))
    return removed


def generate_semantic_route(server):
    result, json_path, csv_path = run_semantic_planning_to_files(
        map_file=server.map_file,
        patrol_area_file=server.output_file,
        output_dir=server.route_output_dir,
        building_safety_buffer_m=server.building_safety_buffer_m,
        default_building_height_m=server.default_building_height_m,
        min_flyable_area_m2=server.min_flyable_area_m2,
        coverage_spacing_m=server.coverage_spacing_m,
        route_edge_margin_m=server.route_edge_margin_m,
        max_off_area_distance_m=server.max_off_area_distance_m,
        sensor_coverage_radius_m=server.sensor_coverage_radius_m,
        min_coverage_contribution_m=server.min_coverage_contribution_m,
        max_connection_length_m=server.max_connection_length_m,
        simplify_tolerance_m=server.simplify_tolerance_m,
        max_2opt_iterations=server.max_2opt_iterations,
        disable_2opt_if_strokes_gt=server.disable_2opt_if_strokes_gt,
        min_patrol_task_length_m=server.min_patrol_task_length_m,
        building_perimeter_offset_m=server.building_perimeter_offset_m,
        open_area_sweep_spacing_m=server.open_area_sweep_spacing_m,
        connector_grid_resolution_m=server.connector_grid_resolution_m,
        max_patrol_tasks=server.max_patrol_tasks,
        altitude_m=server.altitude_m,
        speed_mps=server.speed_mps,
    )
    html_path, _, _, _ = create_visualization(
        map_file=server.map_file,
        patrol_area_file=server.output_file,
        waypoint_file=json_path,
        output_file=server.visualization_file,
    )
    output_dir = resolve_project_path(server.route_output_dir)
    waypoint_payload = json.loads(json_path.read_text(encoding="utf-8"))
    route_points = [
        {
            "index": int(item.get("index", idx)),
            "latitude": float(item["latitude"]),
            "longitude": float(item["longitude"]),
        }
        for idx, item in enumerate(waypoint_payload.get("waypoints", []))
    ]

    def load_geojson(name):
        path = output_dir / name
        if not path.exists():
            return {"type": "FeatureCollection", "features": []}
        return json.loads(path.read_text(encoding="utf-8"))

    return {
        "waypoint_json": str(json_path),
        "waypoint_csv": str(csv_path),
        "visualization": str(html_path),
        "route_points": len(result.route_wgs84),
        "route": route_points,
        "static_obstacles": len(result.static_obstacles_local),
        "obstacle_buffers": len(result.obstacle_buffers_local),
        "flyable_area_empty": result.flyable_area_local.is_empty,
        "static_obstacles_geojson": load_geojson("static_obstacles.geojson"),
        "static_obstacle_buffers_geojson": load_geojson("static_obstacle_buffers.geojson"),
        "flyable_area_geojson": load_geojson("flyable_area.geojson"),
        "target_area_geojson": load_geojson("target_area.geojson"),
        "coverage_target_area_geojson": load_geojson("coverage_target_area.geojson"),
        "planning_airspace_geojson": load_geojson("planning_airspace.geojson"),
        "transit_segments_geojson": load_geojson("transit_segments.geojson"),
        "patrol_diagnostics": result.diagnostics or {},
    }


class PatrolAreaSelectorHandler(BaseHTTPRequestHandler):
    server_version = "PatrolAreaSelector/0.1"

    def _send_json(self, payload, status=200):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_file(self, path, content_type):
        body = path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self):
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0:
            return {}
        return json.loads(self.rfile.read(length).decode("utf-8"))

    def log_message(self, fmt, *args):
        print(f"[patrol_area_selector] {self.address_string()} - {fmt % args}")

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path in ("/", "/index.html", "/patrol_area_selector.html"):
            self._send_file(self.server.html_file, "text/html; charset=utf-8")
            return
        if parsed.path == "/api/config":
            self._send_json(
                {
                    "map_file": str(resolve_project_path(self.server.map_file)),
                    "output": str(resolve_project_path(self.server.output_file)),
                    "bounds": self.server.bounds,
                }
            )
            return
        if parsed.path == "/api/patrol_area":
            path = resolve_project_path(self.server.output_file)
            if path.exists():
                self._send_json(json.loads(path.read_text(encoding="utf-8")))
            else:
                self._send_json({"boundary": []})
            return
        self.send_error(404, "not found")

    def do_POST(self):
        parsed = urllib.parse.urlparse(self.path)
        try:
            if parsed.path == "/api/patrol_area":
                request = self._read_json()
                payload = patrol_area_payload(
                    request.get("area_name", "custom_patrol_area"),
                    request.get("boundary", []),
                )
                path = write_json(self.server.output_file, payload)
                removed = invalidate_route_outputs(self.server.route_output_files)
                route = None
                route_error = None
                if self.server.auto_generate_route:
                    try:
                        route = generate_semantic_route(self.server)
                    except Exception as exc:
                        route_error = str(exc)
                self._send_json(
                    {
                        "ok": True,
                        "path": str(path),
                        "vertices": len(payload["boundary"]),
                        "invalidated": removed,
                        "route_ok": route is not None,
                        "route": route,
                        "route_error": route_error,
                    }
                )
                return
            if parsed.path in ("/api/generate_route", "/api/visualize"):
                route = generate_semantic_route(self.server)
                self._send_json({"ok": True, "route": route})
                return
        except Exception as exc:
            self._send_json({"ok": False, "error": str(exc)}, status=400)
            return
        self.send_error(404, "not found")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", default=DEFAULT_PLANNING.map_file, dest="map_file")
    parser.add_argument("--output", default=DEFAULT_PLANNING.patrol_area_file)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--html", default=str(DEFAULT_HTML))
    parser.add_argument("--waypoints", default=DEFAULT_PLANNING.waypoint_file)
    parser.add_argument("--route-output", default=DEFAULT_PLANNING.route_output_dir)
    parser.add_argument(
        "--visualization",
        default=DEFAULT_PLANNING.visualization_file,
    )
    parser.add_argument("--coverage-spacing", type=float, default=DEFAULT_PLANNING.coverage_spacing_m)
    parser.add_argument("--route-edge-margin", type=float, default=DEFAULT_PLANNING.route_edge_margin_m)
    parser.add_argument("--max-off-area-distance", type=float, default=DEFAULT_PLANNING.max_off_area_distance_m)
    parser.add_argument("--sensor-coverage-radius", type=float, default=DEFAULT_PLANNING.sensor_coverage_radius_m)
    parser.add_argument("--min-coverage-contribution", type=float, default=DEFAULT_PLANNING.min_coverage_contribution_m)
    parser.add_argument("--max-connection-length", type=float, default=DEFAULT_PLANNING.max_connection_length_m)
    parser.add_argument("--simplify-tolerance", type=float, default=DEFAULT_PLANNING.simplify_tolerance_m)
    parser.add_argument("--max-2opt-iterations", type=int, default=DEFAULT_PLANNING.max_2opt_iterations)
    parser.add_argument("--disable-2opt-if-strokes-gt", type=int, default=DEFAULT_PLANNING.disable_2opt_if_strokes_gt)
    parser.add_argument("--min-patrol-task-length", type=float, default=DEFAULT_PLANNING.min_patrol_task_length_m)
    parser.add_argument("--building-perimeter-offset", type=float, default=DEFAULT_PLANNING.building_perimeter_offset_m)
    parser.add_argument("--open-area-sweep-spacing", type=float, default=DEFAULT_PLANNING.open_area_sweep_spacing_m)
    parser.add_argument("--connector-grid-resolution", type=float, default=DEFAULT_PLANNING.connector_grid_resolution_m)
    parser.add_argument("--max-patrol-tasks", type=int, default=DEFAULT_PLANNING.max_patrol_tasks)
    parser.add_argument("--building-buffer", type=float, default=DEFAULT_PLANNING.building_safety_buffer_m)
    parser.add_argument("--default-building-height", type=float, default=DEFAULT_PLANNING.default_building_height_m)
    parser.add_argument("--min-flyable-area", type=float, default=DEFAULT_PLANNING.min_flyable_area_m2)
    parser.add_argument("--altitude", type=float, default=DEFAULT_PLANNING.altitude_m)
    parser.add_argument("--speed", type=float, default=DEFAULT_PLANNING.speed_mps)
    parser.add_argument(
        "--no-auto-generate",
        action="store_true",
        help="Only save patrol_area.json; do not generate semantic route on Save.",
    )
    args = parser.parse_args()

    html_file = resolve_project_path(args.html)
    if not html_file.exists():
        raise FileNotFoundError(html_file)

    server = ThreadingHTTPServer((args.host, args.port), PatrolAreaSelectorHandler)
    server.map_file = args.map_file
    server.output_file = args.output
    server.html_file = html_file
    server.waypoints_file = args.waypoints
    server.route_output_dir = args.route_output
    server.visualization_file = args.visualization
    server.coverage_spacing_m = args.coverage_spacing
    server.route_edge_margin_m = args.route_edge_margin
    server.max_off_area_distance_m = args.max_off_area_distance
    server.sensor_coverage_radius_m = args.sensor_coverage_radius
    server.min_coverage_contribution_m = args.min_coverage_contribution
    server.max_connection_length_m = args.max_connection_length
    server.simplify_tolerance_m = args.simplify_tolerance
    server.max_2opt_iterations = args.max_2opt_iterations
    server.disable_2opt_if_strokes_gt = args.disable_2opt_if_strokes_gt
    server.min_patrol_task_length_m = args.min_patrol_task_length
    server.building_perimeter_offset_m = args.building_perimeter_offset
    server.open_area_sweep_spacing_m = args.open_area_sweep_spacing
    server.connector_grid_resolution_m = args.connector_grid_resolution
    server.max_patrol_tasks = args.max_patrol_tasks
    server.building_safety_buffer_m = args.building_buffer
    server.default_building_height_m = args.default_building_height
    server.min_flyable_area_m2 = args.min_flyable_area
    server.altitude_m = args.altitude
    server.speed_mps = args.speed
    server.auto_generate_route = not args.no_auto_generate
    server.route_output_files = [
        Path(args.route_output) / "uav_waypoints.json",
        Path(args.route_output) / "uav_waypoints.csv",
        Path(args.route_output) / "semantic_uav_waypoints.json",
        Path(args.route_output) / "semantic_uav_waypoints.csv",
        Path(args.route_output) / "target_area.geojson",
        Path(args.route_output) / "coverage_target_area.geojson",
        Path(args.route_output) / "planning_airspace.geojson",
        Path(args.route_output) / "transit_segments.geojson",
        Path(args.route_output) / "patrol_diagnostics.json",
        Path(args.route_output) / "static_obstacles.geojson",
        Path(args.route_output) / "static_obstacle_buffers.geojson",
        Path(args.route_output) / "flyable_area.geojson",
        Path(args.route_output) / "dji_mission_draft.json",
        Path(args.route_output) / "dji_mission.kmz",
        Path(args.route_output) / "dji_mission_validation.log",
        args.visualization,
    ]
    server.bounds = read_osm_bounds(args.map_file)

    print(f"Open http://localhost:{args.port} to draw patrol area.")
    print(f"Map: {resolve_project_path(args.map_file)}")
    print(f"Output: {resolve_project_path(args.output)}")
    print(f"Route output: {resolve_project_path(args.route_output)}")
    print(
        "Save will auto-generate semantic coverage route."
        if server.auto_generate_route
        else "Save will only write patrol_area.json."
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped patrol_area_selector.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
