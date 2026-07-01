from shapely.geometry import LineString, Point, Polygon, box

from uavpatrol_navigation.planning_defaults import DEFAULT_PLANNING
from uavpatrol_navigation.semantic_coverage_planner import (
    compute_semantic_coverage_path,
)


def test_semantic_route_keeps_long_sweeps_sparse():
    target = Polygon([(0, 0), (80, 0), (80, 30), (0, 30)])
    route = compute_semantic_coverage_path(
        target,
        planning_airspace_local=target,
        coverage_spacing_m=10.0,
        route_edge_margin_m=3.0,
        obstacle_buffers_local=[],
    )

    assert len(route) <= 18
    assert (0.0, 5.0) in route
    assert (80.0, 5.0) in route
    assert (40.0, 5.0) not in route


def test_semantic_route_adds_inset_boundary_observation():
    route = compute_semantic_coverage_path(
        Polygon([(0, 0), (80, 0), (80, 30), (0, 30)]),
        coverage_spacing_m=10.0,
        route_edge_margin_m=3.0,
        obstacle_buffers_local=[],
    )

    assert (0.75, 0.75) in route
    assert (79.25, 29.25) in route


def test_semantic_route_stays_inside_drawn_boundary_by_default():
    assert DEFAULT_PLANNING.max_off_area_distance_m == 0.0
    target = box(0, 0, 80, 30)

    route = compute_semantic_coverage_path(
        target,
        coverage_spacing_m=10.0,
        route_edge_margin_m=3.0,
        obstacle_buffers_local=[],
    )

    segments = [LineString([a, b]) for a, b in zip(route, route[1:])]
    assert route
    assert all(target.covers(Point(point)) for point in route)
    assert all(target.covers(segment) for segment in segments)


def test_planning_airspace_remains_hard_boundary_around_obstacle():
    target_area = box(0, 0, 30, 20)
    hard_obstacles = box(13, -4, 17, 24)
    coverage_target = target_area.difference(hard_obstacles)
    planning_airspace = target_area.difference(hard_obstacles)

    route = compute_semantic_coverage_path(
        coverage_target,
        planning_airspace_local=planning_airspace,
        hard_obstacles_local=hard_obstacles,
        coverage_spacing_m=8.0,
        route_edge_margin_m=0.0,
        max_connection_length_m=80.0,
        max_2opt_iterations=20,
    )

    segments = [LineString([a, b]) for a, b in zip(route, route[1:])]
    outside_segments = [segment for segment in segments if not target_area.covers(segment)]

    assert route
    assert not outside_segments
    assert all(planning_airspace.covers(segment) for segment in segments)
    assert all(not segment.intersects(hard_obstacles) for segment in segments)
