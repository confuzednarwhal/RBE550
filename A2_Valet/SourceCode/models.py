import numpy as np
from dataclasses import dataclass
import math
import heapq
from typing import Dict, List, Tuple, Set
import random
from world import Pose, GoalRegion, map


@dataclass
class Plan:
    success: bool
    path: List[Pose]
    cost: float
    expanded: int
    reason: str = ""

@dataclass(frozen=True)
class vehicle:
    goal_cells: Pose
    pos_cells: Tuple[Tuple[int,int], Tuple[int,int], int]
    pos: Pose = (3.0, 6.0, 0.0)
    at_goal: bool = False




"""
class planner:
    def __init__(self, headings: int):
        self.headings = headings

    @staticmethod
    def reconstruct(came_from: Dict[Pose, Pose], goal: Pose) -> List[Pose]:
        path = [goal]
        cur = goal
        while cur in came_from:
            cur = came_from[cur]
            path.append(cur)
        path.reverse()
        return path

    @staticmethod
    def angle_diff(theta: float, alpha: float) -> float:
        beta = (theta - alpha + math.pi) % (2.0 * math.pi) - math.pi
        return beta
    
    @staticmethod
    def wrap_angle(theta: float) -> float:
        return theta % (2.0 * math.pi)
    
    @staticmethod
    def nearest_point_box(x: float, y: float, xmin: float, xmax: float, ymin: float, ymax: float) -> Tuple[float, float]:
        new_x = min(max(x, xmin), xmax)
        new_y = min(max(y, ymin), ymax)
        return new_x, new_y
    
    def heading_bin(self, th: float) -> int:
        # map angle to nearest bin index 0..N-1
        th = self.wrap_angle(th)
        step = 2 * math.pi / self.headings
        return int(round(th / step)) % self.headings
    
    def heuristic(self, p: Pose, goal: GoalRegion, w_heading: float) -> float:
        # distance to the goal box (0 if inside)
        nx, ny = self.nearest_point_box(p.x, p.y, goal.x_range[0], goal.x_range[1], goal.y_range[0], goal.y_range[1])
        dx, dy = (p.x - nx), (p.y - ny)
        h_pos = math.hypot(dx, dy)
        # tiny heading term (keeps admissible if w_heading is small)
        dth = abs(self.angle_diff(p.theta, goal.heading))
        return h_pos + w_heading * dth
    
    def is_goal(self, p: Pose, goal: GoalRegion) -> bool:
        x_ok = (goal.x_range[0] <= p.x <= goal.x_range[1])
        y_ok = (goal.y_range[0] <= p.y <= goal.y_range[1])
        th_ok = abs(self.angle_diff(p.theta, goal.heading)) <= goal.heading_tol
        return x_ok and y_ok and th_ok
    

    def astar(self, world: map, car_pos: Pose, w_heading: float = 0.01, max_expansions: int = 200_000) -> Plan:
        start = car_pos
        nodes, adj = getattr(world, "lattice")
        goal: GoalRegion = getattr(world, "goal_region", None)
        _r, _c, sx, sy = world.world_to_nearest_cell_center(start.x, start.y)

        stheta = self.heading_bin(start.theta)      # snap heading to bin angle
        s = Pose(sx, sy, stheta)

        if s not in nodes:
            # If this trips, headings or centers aren’t quantized the same way as the lattice
            return Plan(False, [], float("inf"), 0, "snapped start Pose not in lattice nodes")

        # --- standard A* ---
        open_heap: List[Tuple[float, float, int, Pose]] = []   # (f, h, tie, node)
        came_from: Dict[Pose, Pose] = {}
        g: Dict[Pose, float] = {s: 0.0}
        h0 = self.heuristic(s, goal, w_heading)
        heapq.heappush(open_heap, (h0, h0, 0, s))

        closed: Set[Pose] = set()
        tie = 1
        expansions = 0

        while open_heap:
            _f, _h, _t, u = heapq.heappop(open_heap)
            if u in closed:
                continue
            closed.add(u)
            expansions += 1

            # goal test on pop → optimal with consistent h
            if self.is_goal(u, goal):
                path = self.reconstruct(came_from, u)
                return Plan(True, path, g[u], expansions, "")

            if expansions >= max_expansions:
                return Plan(False, [], float("inf"), expansions, "max_expansions reached")

            # expand neighbors
            for (v, cost, data) in adj.get(u, []):
                # NOTE: v should already be a valid node key in your lattice
                new_g = g[u] + cost
                old_g = g.get(v, float("inf"))
                if new_g < old_g:
                    g[v] = new_g
                    came_from[v] = u
                    hv = self.heuristic(v, goal, w_heading)
                    fv = new_g + hv
                    heapq.heappush(open_heap, (fv, hv, tie, v))
                    tie += 1

        return Plan(False, [], float("inf"), expansions, "open set exhausted (no path)")
"""