"""Central defaults for the UAV patrol planning workflow."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class PlanningDefaults:
    map_file: str = "data/map.osm"
    patrol_area_file: str = "data/input/patrol_area.json"
    route_output_dir: str = "data/output"
    visualization_file: str = "data/output/patrol_route_visualization.html"
    waypoint_file: str = "data/output/uav_waypoints.json"

    coverage_spacing_m: float = 10.0
    building_safety_buffer_m: float = 5.0
    default_building_height_m: float = 20.0
    min_flyable_area_m2: float = 5.0
    route_edge_margin_m: float = 3.0
    max_off_area_distance_m: float = 0.0
    sensor_coverage_radius_m: float = 0.0
    min_coverage_contribution_m: float = 1.0
    max_connection_length_m: float = 150.0
    simplify_tolerance_m: float = 0.5
    max_2opt_iterations: int = 50
    disable_2opt_if_strokes_gt: int = 80
    min_patrol_task_length_m: float = 20.0
    building_perimeter_offset_m: float = 10.0
    open_area_sweep_spacing_m: float = 30.0
    connector_grid_resolution_m: float = 5.0
    max_patrol_tasks: int = 80
    coverage_score_weight: float = 2.0
    semantic_score_weight: float = 1.0
    detour_score_penalty: float = 0.6
    building_perimeter_task_fraction: float = 0.15
    semantic_side_max_detour_m: float = 60.0
    coverage_sweep_task_priority: float = 1.0
    open_area_task_priority: float = 1.15
    road_task_priority: float = 0.25
    footway_task_priority: float = 0.25
    shoreline_task_priority: float = 0.18
    forest_edge_task_priority: float = 0.12
    building_perimeter_task_priority: float = 0.10

    altitude_m: float = 30.0
    altitude_mode: str = "relative_to_takeoff"
    speed_mps: float = 5.0


DEFAULT_PLANNING = PlanningDefaults()
