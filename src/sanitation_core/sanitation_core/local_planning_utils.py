"""Pure grid utilities used by the local avoidance demo."""

import heapq
import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None

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


def distance_field(source_mask: np.ndarray, resolution: float = 1.0) -> np.ndarray:
    """Return approximate Euclidean distance to the nearest source cell."""
    source_mask = np.asarray(source_mask, dtype=bool)
    if not source_mask.any():
        return np.full(source_mask.shape, np.inf, dtype=np.float32)
    if cv2 is not None:
        return cv2.distanceTransform((~source_mask).astype(np.uint8), cv2.DIST_L2, 5) * resolution
    distances = np.full(source_mask.shape, np.inf, dtype=np.float32)
    frontier = []
    for row, col in np.argwhere(source_mask):
        distances[row, col] = 0.0
        heapq.heappush(frontier, (0.0, int(row), int(col)))
    moves = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)),
             (-1, 1, math.sqrt(2.0)), (-1, -1, math.sqrt(2.0)))
    while frontier:
        distance, row, col = heapq.heappop(frontier)
        if distance != distances[row, col]:
            continue
        for dr, dc, step in moves:
            nxt = row + dr, col + dc
            if not in_bounds(nxt, distances.shape):
                continue
            candidate = distance + step * resolution
            if candidate < distances[nxt]:
                distances[nxt] = candidate
                heapq.heappush(frontier, (candidate, *nxt))
    return distances


def obstacle_cost_field(blocked: np.ndarray, preferred_clearance: float, resolution: float) -> np.ndarray:
    """Build a smooth high-near-obstacle traversal cost in the range [0, 1]."""
    distances = distance_field(blocked, resolution)
    scale = max(preferred_clearance, resolution)
    costs = np.exp(-distances / scale).astype(np.float32)
    costs[np.asarray(blocked, dtype=bool)] = 1.0
    return costs


def path_distance_field(shape: Tuple[int, int], path: Sequence[Cell], resolution: float) -> np.ndarray:
    mask = np.zeros(shape, dtype=bool)
    for cell in path:
        if in_bounds(cell, shape):
            mask[cell] = True
    return distance_field(mask, resolution) if mask.any() else np.zeros(shape, dtype=np.float32)


def weighted_astar_grid(
    blocked: np.ndarray,
    start: Cell,
    goal: Cell,
    traversal_cost: Optional[np.ndarray] = None,
    allow_diagonal: bool = True,
    heuristic_weight: float = 2.0,
    obstacle_cost_weight: float = 4.0,
    smoothness_weight: float = 0.4,
    goal_direction_weight: float = 0.2,
    path_change_cost: Optional[np.ndarray] = None,
    path_change_weight: float = 0.0,
) -> List[Cell]:
    """Cost-aware weighted A* with turn, goal-direction, and previous-path penalties."""
    blocked = np.asarray(blocked, dtype=bool)
    if not in_bounds(start, blocked.shape) or not in_bounds(goal, blocked.shape):
        return []
    if blocked[start] or blocked[goal]:
        return []
    traversal = np.zeros(blocked.shape, dtype=np.float32) if traversal_cost is None else np.asarray(traversal_cost)
    path_change = np.zeros(blocked.shape, dtype=np.float32) if path_change_cost is None else np.asarray(path_change_cost)
    moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0)]
    if allow_diagonal:
        root2 = math.sqrt(2.0)
        moves += [(1, 1, root2), (1, -1, root2), (-1, 1, root2), (-1, -1, root2)]
    goal_row, goal_col = goal
    goal_vector = goal_row - start[0], goal_col - start[1]
    goal_norm = math.hypot(*goal_vector)
    if goal_norm > 0.0:
        goal_vector = goal_vector[0] / goal_norm, goal_vector[1] / goal_norm

    def heuristic(cell):
        return heuristic_weight * math.hypot(goal_row - cell[0], goal_col - cell[1])

    direction_penalties = [
        goal_direction_weight * (1.0 - (goal_vector[0] * dr + goal_vector[1] * dc) / step)
        if goal_norm > 0.0 else 0.0
        for dr, dc, step in moves
    ]
    turn_penalties = np.zeros((len(moves), len(moves)), dtype=np.float32)
    for previous_index, (pdr, pdc, previous_step) in enumerate(moves):
        for move_index, (dr, dc, step) in enumerate(moves):
            dot = (pdr * dr + pdc * dc) / (previous_step * step)
            turn_penalties[previous_index, move_index] = smoothness_weight * (
                1.0 - max(-1.0, min(1.0, dot))
            )
    frontier = [(heuristic(start), 0.0, start)]
    best = np.full(blocked.shape, np.inf, dtype=np.float32)
    best[start] = 0.0
    closed = np.zeros(blocked.shape, dtype=bool)
    parent_row = np.full(blocked.shape, -1, dtype=np.int32)
    parent_col = np.full(blocked.shape, -1, dtype=np.int32)
    arrival_move = np.full(blocked.shape, -1, dtype=np.int8)
    while frontier:
        _, cost, current = heapq.heappop(frontier)
        if closed[current] or cost > float(best[current]) + 1e-5:
            continue
        closed[current] = True
        if current == goal:
            break
        previous_move = int(arrival_move[current])
        for move_index, (dr, dc, step) in enumerate(moves):
            nxt = current[0] + dr, current[1] + dc
            if not in_bounds(nxt, blocked.shape) or blocked[nxt] or closed[nxt]:
                continue
            if dr and dc and (blocked[current[0] + dr, current[1]] or blocked[current[0], current[1] + dc]):
                continue
            turn_cost = float(turn_penalties[previous_move, move_index]) if previous_move >= 0 else 0.0
            new_cost = (
                cost + step + obstacle_cost_weight * float(traversal[nxt]) * step
                + turn_cost + direction_penalties[move_index] + path_change_weight * float(path_change[nxt])
            )
            if new_cost < float(best[nxt]):
                best[nxt] = new_cost
                parent_row[nxt], parent_col[nxt] = current
                arrival_move[nxt] = move_index
                heapq.heappush(frontier, (new_cost + heuristic(nxt), new_cost, nxt))
    if not closed[goal]:
        return []
    current = goal
    path = [current]
    while current != start:
        current = int(parent_row[current]), int(parent_col[current])
        path.append(current)
    return list(reversed(path))


def forward_corridor_has_obstacle(
    blocked: np.ndarray,
    start: Cell,
    goal: Cell,
    distance_cells: int,
    corridor_half_width_cells: int,
) -> bool:
    """Check whether the direct forward corridor contains an obstacle."""
    blocked = np.asarray(blocked, dtype=bool)
    vector = np.array([goal[0] - start[0], goal[1] - start[1]], dtype=np.float32)
    norm = float(np.linalg.norm(vector))
    if norm < 1.0:
        return False
    forward = vector / norm
    left = np.array([forward[1], -forward[0]], dtype=np.float32)
    obstacle_cells = np.argwhere(blocked).astype(np.float32)
    if obstacle_cells.size == 0:
        return False
    relative = obstacle_cells - np.asarray(start, dtype=np.float32)
    longitudinal = relative @ forward
    lateral = relative @ left
    return bool(np.any(
        (longitudinal >= 1.0) & (longitudinal <= distance_cells)
        & (np.abs(lateral) <= corridor_half_width_cells)
    ))


def early_avoidance_candidates(
    blocked: np.ndarray,
    start: Cell,
    goal: Cell,
    distance_cells: int,
    corridor_half_width_cells: int,
    clearance_cells: int,
) -> List[Tuple[str, Cell]]:
    """Return clear left/right waypoints around the first obstacle in the forward corridor."""
    blocked = np.asarray(blocked, dtype=bool)
    vector = np.array([goal[0] - start[0], goal[1] - start[1]], dtype=np.float32)
    norm = float(np.linalg.norm(vector))
    if norm < 1.0:
        return []
    forward = vector / norm
    left = np.array([forward[1], -forward[0]], dtype=np.float32)
    obstacle_cells = np.argwhere(blocked).astype(np.float32)
    if obstacle_cells.size == 0:
        return []
    relative = obstacle_cells - np.asarray(start, dtype=np.float32)
    longitudinal = relative @ forward
    lateral = relative @ left
    relevant = (
        (longitudinal >= 1.0) & (longitudinal <= distance_cells)
        & (np.abs(lateral) <= corridor_half_width_cells)
    )
    if not relevant.any():
        return []
    first_obstacle = float(longitudinal[relevant].min())
    distance_to_obstacle = distance_field(blocked)
    forward_distance = min(float(distance_cells), first_obstacle + max(2, clearance_cells))
    lateral_distance = corridor_half_width_cells + clearance_cells
    candidates = []
    for name, sign in (("left", 1.0), ("right", -1.0)):
        point = np.array(start, dtype=np.float32) + forward * forward_distance + left * lateral_distance * sign
        candidate = int(round(point[0])), int(round(point[1]))
        if in_bounds(candidate, blocked.shape) and not blocked[candidate] and distance_to_obstacle[candidate] >= clearance_cells:
            candidates.append((name, candidate))
    return candidates


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
