"""Pure grid utilities used by the local avoidance demo."""

import heapq
import math
from typing import List, Optional, Tuple

import numpy as np

Cell = Tuple[int, int]  # row, column


def inflate_occupied(occupied: np.ndarray, radius_cells: int) -> np.ndarray:
    """Inflate boolean occupied cells with a circular kernel."""
    occupied = np.asarray(occupied, dtype=bool)
    if radius_cells <= 0 or not occupied.any():
        return occupied.copy()
    out = occupied.copy()
    height, width = occupied.shape
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dx * dx + dy * dy > radius_cells * radius_cells:
                continue
            src_y0, src_y1 = max(0, -dy), min(height, height - dy)
            src_x0, src_x1 = max(0, -dx), min(width, width - dx)
            dst_y0, dst_y1 = src_y0 + dy, src_y1 + dy
            dst_x0, dst_x1 = src_x0 + dx, src_x1 + dx
            out[dst_y0:dst_y1, dst_x0:dst_x1] |= occupied[src_y0:src_y1, src_x0:src_x1]
    return out


def world_to_cell(x: float, y: float, origin_x: float, origin_y: float, resolution: float) -> Cell:
    return int(math.floor((y - origin_y) / resolution)), int(math.floor((x - origin_x) / resolution))


def cell_to_world(cell: Cell, origin_x: float, origin_y: float, resolution: float) -> Tuple[float, float]:
    row, col = cell
    return origin_x + (col + 0.5) * resolution, origin_y + (row + 0.5) * resolution


def in_bounds(cell: Cell, shape: Tuple[int, int]) -> bool:
    return 0 <= cell[0] < shape[0] and 0 <= cell[1] < shape[1]


def nearest_free(blocked: np.ndarray, start: Cell) -> Optional[Cell]:
    """Find the nearest free cell using an expanding breadth-first search."""
    if not in_bounds(start, blocked.shape):
        return None
    if not blocked[start]:
        return start
    queue = [start]
    seen = {start}
    for cell in queue:
        row, col = cell
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nxt = row + dr, col + dc
            if not in_bounds(nxt, blocked.shape) or nxt in seen:
                continue
            if not blocked[nxt]:
                return nxt
            seen.add(nxt)
            queue.append(nxt)
    return None


def astar_grid(blocked: np.ndarray, start: Cell, goal: Cell, allow_diagonal: bool = True) -> List[Cell]:
    """Plan a shortest path on a boolean obstacle grid."""
    blocked = np.asarray(blocked, dtype=bool)
    if not in_bounds(start, blocked.shape) or not in_bounds(goal, blocked.shape):
        return []
    if blocked[start] or blocked[goal]:
        return []

    moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0)]
    if allow_diagonal:
        root2 = math.sqrt(2.0)
        moves += [(1, 1, root2), (1, -1, root2), (-1, 1, root2), (-1, -1, root2)]

    def heuristic(cell: Cell) -> float:
        dr, dc = abs(goal[0] - cell[0]), abs(goal[1] - cell[1])
        return math.hypot(dr, dc)

    frontier = [(heuristic(start), 0.0, start)]
    came_from = {}
    best = {start: 0.0}
    while frontier:
        _, cost, current = heapq.heappop(frontier)
        if cost != best.get(current):
            continue
        if current == goal:
            path = [current]
            while current != start:
                current = came_from[current]
                path.append(current)
            return list(reversed(path))
        for dr, dc, step in moves:
            nxt = current[0] + dr, current[1] + dc
            if not in_bounds(nxt, blocked.shape) or blocked[nxt]:
                continue
            if dr and dc and (blocked[current[0] + dr, current[1]] or blocked[current[0], current[1] + dc]):
                continue
            new_cost = cost + step
            if new_cost < best.get(nxt, math.inf):
                best[nxt] = new_cost
                came_from[nxt] = current
                heapq.heappush(frontier, (new_cost + heuristic(nxt), new_cost, nxt))
    return []


def simplify_collinear(path: List[Cell]) -> List[Cell]:
    """Remove interior points that continue in the same grid direction."""
    if len(path) < 3:
        return list(path)
    result = [path[0]]
    previous_direction = None
    for index in range(1, len(path)):
        direction = path[index][0] - path[index - 1][0], path[index][1] - path[index - 1][1]
        if previous_direction is not None and direction != previous_direction:
            result.append(path[index - 1])
        previous_direction = direction
    result.append(path[-1])
    return result
