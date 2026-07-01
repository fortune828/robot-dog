#!/usr/bin/env python3
"""Run OSM semantic constrained UAV coverage planning without ROS."""

import argparse
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PKG_SRC = ROOT / "src" / "uavpatrol_navigation"
if str(PKG_SRC) not in sys.path:
    sys.path.insert(0, str(PKG_SRC))

from uavpatrol_navigation.semantic_coverage_planner import run_semantic_planning_to_files
from uavpatrol_navigation.patrol_visualization import create_visualization
from uavpatrol_navigation.planning_defaults import DEFAULT_PLANNING


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", default=DEFAULT_PLANNING.map_file, dest="map_file")
    parser.add_argument("--area", default=DEFAULT_PLANNING.patrol_area_file)
    parser.add_argument("--output", default=DEFAULT_PLANNING.route_output_dir)
    parser.add_argument("--building-buffer", type=float, default=DEFAULT_PLANNING.building_safety_buffer_m)
    parser.add_argument("--default-building-height", type=float, default=DEFAULT_PLANNING.default_building_height_m)
    parser.add_argument("--min-flyable-area", type=float, default=DEFAULT_PLANNING.min_flyable_area_m2)
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
    parser.add_argument("--altitude", type=float, default=DEFAULT_PLANNING.altitude_m)
    parser.add_argument("--speed", type=float, default=DEFAULT_PLANNING.speed_mps)
    args = parser.parse_args()

    result, json_path, csv_path = run_semantic_planning_to_files(
        map_file=args.map_file,
        patrol_area_file=args.area,
        output_dir=args.output,
        building_safety_buffer_m=args.building_buffer,
        default_building_height_m=args.default_building_height,
        min_flyable_area_m2=args.min_flyable_area,
        coverage_spacing_m=args.coverage_spacing,
        route_edge_margin_m=args.route_edge_margin,
        max_off_area_distance_m=args.max_off_area_distance,
        sensor_coverage_radius_m=args.sensor_coverage_radius,
        min_coverage_contribution_m=args.min_coverage_contribution,
        max_connection_length_m=args.max_connection_length,
        simplify_tolerance_m=args.simplify_tolerance,
        max_2opt_iterations=args.max_2opt_iterations,
        disable_2opt_if_strokes_gt=args.disable_2opt_if_strokes_gt,
        min_patrol_task_length_m=args.min_patrol_task_length,
        building_perimeter_offset_m=args.building_perimeter_offset,
        open_area_sweep_spacing_m=args.open_area_sweep_spacing,
        connector_grid_resolution_m=args.connector_grid_resolution,
        max_patrol_tasks=args.max_patrol_tasks,
        altitude_m=args.altitude,
        speed_mps=args.speed,
    )
    html_path, _, _, _ = create_visualization(
        map_file=args.map_file,
        patrol_area_file=args.area,
        waypoint_file=json_path,
        output_file=Path(args.output) / "patrol_route_visualization.html",
    )

    print(f"target_area: {Path(args.output) / 'target_area.geojson'}")
    print(f"coverage_target_area: {Path(args.output) / 'coverage_target_area.geojson'}")
    print(f"planning_airspace: {Path(args.output) / 'planning_airspace.geojson'}")
    print(f"static_obstacles: {Path(args.output) / 'static_obstacles.geojson'}")
    print(f"static_obstacle_buffers: {Path(args.output) / 'static_obstacle_buffers.geojson'}")
    print(f"transit_segments: {Path(args.output) / 'transit_segments.geojson'}")
    print(f"patrol_diagnostics: {Path(args.output) / 'patrol_diagnostics.json'}")
    print(f"flyable_area_compat: {Path(args.output) / 'flyable_area.geojson'}")
    print(f"waypoints_json: {json_path}")
    print(f"waypoints_csv: {csv_path}")
    print(f"route_points: {len(result.route_wgs84)}")
    print(f"visualization: {html_path}")


if __name__ == "__main__":
    main()
