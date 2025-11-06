# planner_lazy.py
from __future__ import annotations
from dataclasses import dataclass
from math import cos, sin, pi
import math
import heapq
from typing import Dict, Tuple, List, Optional

# Try to reuse EdgeData from your codebase if it exists
try:
    from world import EdgeData  # type: ignore
except Exception:
    @dataclass(frozen=True)
    class EdgeData:  # minimal local fallback
        dtheta: float
        arc_len: float

# Import your Pose (continuous) and world type
from world import Pose  # type: ignore

@dataclass
class AStarResult:
    path: List[Pose]
    expanded: int
    cost: float
    success: bool

class AStarPlanner:
    """
    Lazy-lattice A*: neighbors are generated on demand via your world's
    primitive library + ε-hash + capsule collision gates.
    """
    def __init__(
        self,
        world,                      # your map/world instance
        wheelbase: float = 2.5,
        steering_angles_deg = (-45.0, -25.0, 0.0, +25.0, +45.0),
        lengths = (-3.0, 3.0, -1.0, 1.0, 1.5, -1.5, 0.5, -0.5)       # start simple; add more if needed
    ):
        self.world = world
        self.wheelbase = wheelbase

        # Build the SAME primitive set your lattice uses
        self.primitive_lib = world.gen_primitive_lib(
            steer_ang_deg=steering_angles_deg,
            arc_l=lengths,
            wheelbase=wheelbase,
        )

        # ε-hash node registry and neighbor cache
        self.nodes_by_key: Dict[Tuple[int,int,int], Pose] = {}
        self.neigh_cache: Dict[Tuple[int,int,int], List[Tuple[Pose, float, EdgeData]]] = {}

        # Precompute bounds in meters for a fast in-bounds check
        self.world_w = self.world.num_cols * self.world.cell_size
        self.world_h = self.world.num_rows * self.world.cell_size

    # --------- utilities ---------
    def in_world_bounds(self, x: float, y: float) -> bool:
        eps = 1e-9
        return (-eps <= x <= self.world_w + eps) and (-eps <= y <= self.world_h + eps)

    def key_of(self, p: Pose) -> Tuple[int,int,int]:
        return self.world.pose_key_from_vals(p.x, p.y, p.theta)

    def get_or_make_node(self, x: float, y: float, th: float) -> Pose:
        return self.world.get_or_make_node(x, y, th, self.nodes_by_key)

    # --------- goal test / heuristic ---------
    def in_goal_region(self, p: Pose) -> bool:
        gr = getattr(self.world, "goal_region", None)

        if gr is None:
            return False
        x_ok = (gr.x_range[0] <= p.x <= gr.x_range[1])
        y_ok = (gr.y_range[0] <= p.y <= gr.y_range[1])
        # smallest signed angle diff
        dth = ((p.theta - gr.heading + math.pi) % (2*math.pi)) - math.pi
        th_ok = abs(dth) <= gr.heading_tol
        return x_ok and y_ok and th_ok

    def heuristic(self, p: Pose) -> float:
        """Admissible-enough: distance to goal box + small heading penalty."""
        gr = getattr(self.world, "goal_region", None)
        if gr is None:
            return 0.0
        # dx, dy to the rectangle (0 if already inside range)
        dx = 0.0
        if p.x < gr.x_range[0]: dx = gr.x_range[0] - p.x
        elif p.x > gr.x_range[1]: dx = p.x - gr.x_range[1]
        dy = 0.0
        if p.y < gr.y_range[0]: dy = gr.y_range[0] - p.y
        elif p.y > gr.y_range[1]: dy = p.y - gr.y_range[1]
        dpos = math.hypot(dx, dy)
        dth = ((p.theta - gr.heading + math.pi) % (2*math.pi)) - math.pi
        return dpos + 0.2 * abs(dth)

    # --------- lazy neighbor generation (with cache) ---------
    def neighbors(self, u: Pose) -> List[Tuple[Pose, float, EdgeData]]:
        ku = self.key_of(u)
        if ku in self.neigh_cache:
            return self.neigh_cache[ku]

        out: List[Tuple[Pose, float, EdgeData]] = []
        s_th = u.theta

        for (endpoint_local, dtheta, arc_len) in self.primitive_lib:
            # Transform local endpoint by u.theta
            wx = endpoint_local[0] * cos(s_th) - endpoint_local[1] * sin(s_th)
            wy = endpoint_local[0] * sin(s_th) + endpoint_local[1] * cos(s_th)
            x_new, y_new = u.x + wx, u.y + wy
            th_new = self.world.wrap_angle(s_th + dtheta)

            # bounds + swept collision check
            if not self.in_world_bounds(x_new, y_new):
                continue
            if not self.world.is_edge_free(u, dtheta=dtheta, arc_l=arc_len, wheelbase=self.wheelbase):
                continue

            v = self.get_or_make_node(x_new, y_new, th_new)
            cost = abs(arc_len) + 0.05 * abs(dtheta)
            meta = EdgeData(dtheta=dtheta, arc_len=arc_len)
            out.append((v, cost, meta))

        self.neigh_cache[ku] = out
        return out

    # --------- public A* ---------
    def plan(self, start: Pose, max_iters: int = 100000) -> AStarResult:
        """Returns an AStarResult with path in Pose (meters), or success=False if no plan."""
        # seed the start in the ε-registry
        s = self.get_or_make_node(start.x, start.y, start.theta)
        ks = self.key_of(s)


        # A* sets
        open_heap: List[Tuple[float, float, Tuple[int,int,int], Pose]] = []
        heapq.heappush(open_heap, (self.heuristic(s), 0.0, ks, s))
        g: Dict[Tuple[int,int,int], float] = {ks: 0.0}
        came: Dict[Tuple[int,int,int], Tuple[Tuple[int,int,int], Pose, EdgeData]] = {}

        expanded = 0

        while open_heap and expanded < max_iters:
            f, g_u, ku, u = heapq.heappop(open_heap)
            expanded += 1

            # gr = self.world.goal_region
            # print("Goal box:", gr.x_range, gr.y_range, "ψ", gr.heading, "±", gr.heading_tol)

            # print("Node at check (m):", u.x, u.y, "θ:", u.theta)
            # dx = 0.0 if gr.x_range[0] <= u.x <= gr.x_range[1] else min(abs(u.x-gr.x_range[0]), abs(u.x-gr.x_range[1]))
            # dy = 0.0 if gr.y_range[0] <= u.y <= gr.y_range[1] else min(abs(u.y-gr.y_range[0]), abs(u.y-gr.y_range[1]))
            # dth = ((u.theta - gr.heading + math.pi) % (2*math.pi)) - math.pi
            # print("dx, dy, |dθ|:", dx, dy, abs(dth))


            # goal test on the continuous pose
            if self.in_goal_region(u):
                print("Checking (meters):", u.x, u.y, u.theta)
                # reconstruct (continuous) path
                path: List[Pose] = [u]
                k = ku
                while k in came:
                    k_prev, u_prev, _meta = came[k]
                    path.append(u_prev)
                    k = k_prev
                path.reverse()
                print(path)
                return AStarResult(path=path, expanded=expanded, cost=g_u, success=True)

            # expand neighbors lazily
            for (v, cost, meta) in self.neighbors(u):
                kv = self.key_of(v)
                tentative = g_u + cost
                if tentative < g.get(kv, float("inf")):
                    g[kv] = tentative
                    came[kv] = (ku, u, meta)
                    f_v = tentative + self.heuristic(v)
                    heapq.heappush(open_heap, (f_v, tentative, kv, v))

        return AStarResult(path=[], expanded=expanded, cost=float("inf"), success=False)
