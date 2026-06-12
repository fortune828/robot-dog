import numpy as np

from sanitation_core.local_planning_utils import (
    astar_grid,
    cell_to_world,
    distance_field,
    early_avoidance_candidates,
    forward_corridor_has_obstacle,
    inflate_occupied,
    nearest_free,
    obstacle_cost_field,
    simplify_collinear,
    weighted_astar_grid,
    world_to_cell,
)


def test_inflation_is_circular_and_preserves_source():
    grid = np.zeros((7, 7), dtype=bool)
    grid[3, 3] = True
    inflated = inflate_occupied(grid, 2)
    assert inflated[3, 3]
    assert inflated[3, 5]
    assert not inflated[1, 1]


def test_world_cell_round_trip_uses_cell_center():
    cell = world_to_cell(1.05, -0.95, 0.0, -3.0, 0.1)
    x, y = cell_to_world(cell, 0.0, -3.0, 0.1)
    assert abs(x - 1.05) < 1e-9
    assert abs(y + 0.95) < 1e-9


def test_nearest_free_moves_from_blocked_cell():
    blocked = np.zeros((5, 5), dtype=bool)
    blocked[2, 2] = True
    assert nearest_free(blocked, (2, 2)) != (2, 2)


def test_astar_routes_around_wall():
    blocked = np.zeros((7, 7), dtype=bool)
    blocked[:6, 3] = True
    path = astar_grid(blocked, (3, 1), (3, 5), allow_diagonal=True)
    assert path[0] == (3, 1)
    assert path[-1] == (3, 5)
    assert all(not blocked[cell] for cell in path)


def test_astar_does_not_cut_blocked_corner():
    blocked = np.zeros((3, 3), dtype=bool)
    blocked[0, 1] = True
    blocked[1, 0] = True
    assert astar_grid(blocked, (0, 0), (1, 1), allow_diagonal=True) == []


def test_simplify_collinear_keeps_turns():
    path = [(0, 0), (0, 1), (0, 2), (1, 2), (2, 2)]
    assert simplify_collinear(path) == [(0, 0), (0, 2), (2, 2)]


def test_obstacle_cost_decays_with_clearance():
    blocked = np.zeros((9, 9), dtype=bool)
    blocked[4, 4] = True
    costs = obstacle_cost_field(blocked, preferred_clearance=2.0, resolution=1.0)
    assert costs[4, 4] == 1.0
    assert costs[4, 5] > costs[4, 7] > 0.0


def test_weighted_astar_keeps_more_clearance_than_binary_astar():
    blocked = np.zeros((15, 15), dtype=bool)
    blocked[7, 7] = True
    start, goal = (7, 1), (7, 13)
    binary = astar_grid(blocked, start, goal)
    weighted = weighted_astar_grid(
        blocked, start, goal, obstacle_cost_field(blocked, 3.0, 1.0),
        obstacle_cost_weight=12.0, smoothness_weight=0.5,
    )
    clearance = distance_field(blocked)
    assert min(clearance[cell] for cell in weighted) > min(clearance[cell] for cell in binary)


def test_early_avoidance_proposes_both_clear_sides():
    blocked = np.zeros((21, 21), dtype=bool)
    blocked[10, 8] = True
    start, goal = (10, 1), (10, 19)
    assert forward_corridor_has_obstacle(blocked, start, goal, 12, 1)
    candidates = early_avoidance_candidates(blocked, start, goal, 12, 1, 3)
    assert {side for side, _ in candidates} == {"left", "right"}


def test_early_avoidance_full_wall_has_no_complete_route():
    blocked = np.zeros((21, 21), dtype=bool)
    blocked[:, 8] = True
    start, goal = (10, 1), (10, 19)
    candidates = early_avoidance_candidates(blocked, start, goal, 12, 1, 3)
    assert candidates
    for _, waypoint in candidates:
        first = weighted_astar_grid(blocked, start, waypoint)
        second = weighted_astar_grid(blocked, waypoint, goal)
        assert not first or not second
