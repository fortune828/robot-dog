import numpy as np

from sanitation_core.local_planning_utils import astar_grid, cell_to_world, inflate_occupied, nearest_free, simplify_collinear, world_to_cell


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
