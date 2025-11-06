from __future__ import annotations
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from typing import Dict, List, Tuple, Iterable, Set
import random
import math

from dataclasses import dataclass
from math import cos, sin, tan, pi, hypot, isclose, floor

@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    theta: float

@dataclass(frozen=True)
class EdgeData:
    dtheta: float
    arc_len: float 

@dataclass()
class GoalRegion:
    x_range: Tuple[float,float]
    y_range: Tuple[float,float]
    heading: float
    heading_tol: float

Edge = Tuple[Pose, float, EdgeData]



TETRIMINOS: Dict[str, np.array] = {
    "I": np.array([[1, 1, 1, 1]], dtype=np.uint8),
    "O": np.array([[1, 1],
                   [1, 1]], dtype=np.uint8),
    "T": np.array([[1, 1, 1],
                   [0, 1, 0]], dtype=np.uint8),
    "Z": np.array([[1, 1, 0],
                   [0, 1, 1]], dtype=np.uint8),
    "L": np.array([[0, 0, 1],
                   [1, 1, 1]], dtype=np.uint8),
}


# handles all the shape rotations for tetriminos
def rotate_shape(shape: np.array) -> np.array:
    rotations:List[np.array] = []
    curr = shape

    curr = shape.copy()
    for _ in range(4):
        if not any(np.array_equal(curr, r) for r in rotations):
            rotations.append(curr.copy())
        curr = np.rot90(curr)

    return rotations  # CHANGED: return List[np.array] for clarity

# builds dictionary of the rotations once, so no need to recompute each time
def build_rotations(shapes: Dict[str, np.array]) -> Dict[str, List[np.array]]:
    rotations: Dict[str, List[np.array]] = {}

    for name, shape in shapes.items():
        rotations[name] = rotate_shape(shape)

    return rotations

# dict of all the potential rotations
TETRIMINO_ROTATIONS = build_rotations(TETRIMINOS)


class map:
    def __init__(self):
        self.cell_size: float = 3.0
        self.num_rows: int = 12
        self.num_cols: int = 12
        self.density: float = 0.15
        self.field = np.zeros((self.num_rows, self.num_cols), dtype=np.uint8)
        self.rng = np.random.default_rng() 
        self.rotations: Dict[str, List[np.array]] = TETRIMINO_ROTATIONS

        self.lattice: Tuple[Set[Pose], Dict[Pose, List[Edge]]] = []
        self.goal_cells: Tuple[Tuple[int,int], Tuple[int,int], int] = []
        self.goal_region: GoalRegion | None = None

        self.vehicle_w: float = 3.0
        self.vehicle_l: float = 6.0       
        self.obstacle_clearance: float = 0.5
        self.sample_step: float = max(0.25, 0.4 * self.cell_size)

        # === ε-quantization for continuous graph lookups ===  # NEW
        self.res_xy: float = 1.0 * self.cell_size   # ~0.6–1.0 × cell_size works well
        self.res_th_deg: float = 10.0
        self.res_th_rad: float = math.radians(self.res_th_deg)
       
    
    @staticmethod
    def wrap_angle(th: float) -> float:
        return th % (2.0 * math.pi)

    def heading_bin(self, th: float, N: int) -> int:
        # map angle to nearest bin index 0..N-1
        th = self.wrap_angle(th)
        step = 2 * math.pi / N
        return int(round(th / step)) % N

    def bin_to_angle(self, th: float, N: int) -> float:
        step = 2 * math.pi / N
        b = int(round(self.wrap_angle(th) / step)) % N
        return b * step 
    
    @staticmethod
    def snap_pose(x: float, y: float, theta: float) -> Pose:
        t = (theta % (2 * math.pi))
        return Pose(round(x, 2), round(y, 2), round(t, 2))
    
    def world_to_cell(self, x: float, y:float) -> Tuple[int,int]:
        r = int(y // self.cell_size)
        c = int(x // self.cell_size)
        return r, c

    def pose_key_from_vals(self, x: float, y: float, th: float) -> tuple[int,int,int]:   # NEW
        th_wrapped = self.wrap_angle(th)
        kx = int(round(x / self.res_xy))
        ky = int(round(y / self.res_xy))
        kh = int(round(th_wrapped / self.res_th_rad))
        return (kx, ky, kh)

    def get_or_make_node(                                              # NEW
        self, x: float, y: float, th: float,
        nodes_by_key: dict[tuple[int,int,int], Pose]
    ) -> Pose:
        k = self.pose_key_from_vals(x, y, th)
        v = nodes_by_key.get(k)
        if v is None:
            v = Pose(x, y, self.wrap_angle(th))
            nodes_by_key[k] = v
        return v


    # center of cell (r,c) in world meters
    def cell_center_to_world(self, r: int, c: int) -> tuple[float, float]:
        return (c + 0.5) * self.cell_size, (r + 0.5) * self.cell_size

    # snap any (x,y) to nearest cell center; returns (r,c,xc,yc)
    def world_to_nearest_cell_center(self, x: float, y: float) -> tuple[int, int, float, float]:
        c = int(round(x / self.cell_size - 0.5))
        r = int(round(y / self.cell_size - 0.5))
        r = max(0, min(self.num_rows - 1, r))
        c = max(0, min(self.num_cols - 1, c))
        xc, yc = self.cell_center_to_world(r, c)
        return r, c, xc, yc

    def is_occupied(self, r, c):
        H, W = self.field.shape
        return (r < 0) or (r >= H) or (c < 0) or (c >= W) or (self.field[r, c] != 0)

    
    def occupied_in_radius(self, x: float, y: float, radius_m: float) -> bool: 
        """Return True if any *in-bounds* obstacle cell center lies within radius_m of (x,y)."""
        H, W = self.field.shape
        r0, c0 = self.world_to_cell(x, y)

        rr = int(y // self.cell_size)
        cc = int(x // self.cell_size)
        if 0 <= rr < H and 0 <= cc < W and self.field[rr, cc] != 0:
            return True

        # search window in cell indices, clipped to the grid (unchanged)
        k = int(np.ceil(radius_m / self.cell_size))
        rmin = max(0, rr - k); rmax = min(H - 1, rr + k)
        cmin = max(0, cc - k); cmax = min(W - 1, cc + k)

        half = 0.5 * self.cell_size
        r2 = radius_m * radius_m

        for rcell in range(rmin, rmax + 1):
            for ccell in range(cmin, cmax + 1):
                if self.field[rcell, ccell] == 0:
                    continue  # free

                # obstacle cell rectangle bounds in meters
                cx = (ccell + 0.5) * self.cell_size
                cy = (rcell + 0.5) * self.cell_size
                xmin, xmax = cx - half, cx + half
                ymin, ymax = cy - half, cy + half

                # squared distance from point (x,y) to the rectangle
                dx = 0.0
                if x < xmin: dx = xmin - x
                elif x > xmax: dx = x - xmax

                dy = 0.0
                if y < ymin: dy = ymin - y
                elif y > ymax: dy = y - ymax

                if dx*dx + dy*dy <= r2:
                    return True

        return False

    
    def capsule_collides(self, x: float, y: float, theta: float) -> bool:
        W = self.vehicle_w
        L = self.vehicle_l
        r = 0.5 * W + self.obstacle_clearance               # capsule radius
        straight = max(0.0, L - 2.0 * r)                    # center-segment length

        ct, st = math.cos(theta), math.sin(theta)
        hx, hy = 0.5 * straight * ct, 0.5 * straight * st
        x0, y0 = x - hx, y - hy                              # rear cap center
        x1, y1 = x + hx, y + hy                              # front cap center

        # sample discs along center segment; spacing ~ 0.6*r gives tight coverage
        n = max(1, int(math.ceil(straight / max(1e-6, 0.6 * r))))
        for i in range(n + 1):
            t = 0.0 if n == 0 else i / n
            cx = x0 + t * (x1 - x0)
            cy = y0 + t * (y1 - y0)
            if self.occupied_in_radius(cx, cy, r):
                return True

        # Also check the end-caps explicitly
        if self.occupied_in_radius(x0, y0, r): return True
        if self.occupied_in_radius(x1, y1, r): return True
        return False

    # Replace your sampler with curvature from the primitive:
    def sample_arc_points(self, s: Pose, dtheta: float, arc_l: float,
                        wheelbase: float, step: float) -> List[Tuple[float, float, float]]:
        n = max(2, int(math.ceil(abs(arc_l) / max(1e-6, step))))
        ds = arc_l / n  # signed (works for reverse)
        # curvature from primitive itself
        k = 0.0 if abs(arc_l) < 1e-9 else (dtheta / arc_l)

        x, y, th = s.x, s.y, s.theta
        pts = []
        for _ in range(n):
            if abs(k) < 1e-10:
                # straight
                x += ds * math.cos(th)
                y += ds * math.sin(th)
            else:
                # turning
                th += ds * k
                x  += ds * math.cos(th)
                y  += ds * math.sin(th)
            pts.append((x, y, th))
        return pts

    def is_edge_free(self, s: Pose, dtheta: float, arc_l: float, wheelbase: float) -> bool:
        samples = self.sample_arc_points(
            s, dtheta=dtheta, arc_l=arc_l,
            wheelbase=wheelbase, step=self.sample_step
        )
        for (x, y, th) in samples:
            if self.capsule_collides(x, y, th):
                return False
        return True
    # generates map at specific density
    def generate_map(self) -> np.array:
        curr_density = 0
        # field = np.zeros((fieldSize_row, fieldSize_col))
    
    
        # picks a random rotated tetrimino
        def pick_tetrimino():
            name = self.rng.choice(list(self.rotations.keys()))
            rots = self.rotations[name]

            idx = self.rng.integers(len(rots))
            return rots[idx]


            # places shape in random spot in field, handles overlapping shapes
        def place_shape(shape, row, col):
            h, w = shape.shape
            self.field[row:row+h, col:col+w] = np.maximum(self.field[row:row+h, col:col+w], shape)
            # field[row:row+h, col:col+w] = shape

            # checks to see if shape fits the bounds of the field 
        def can_place(shape, row, col) -> bool:
            h, w = shape.shape
            H, W = self.field.shape
            return (row + h < H and col + w < W)

        # places shapes until desired density is reached
        while curr_density <= self.density:
            curr_density = np.sum(self.field) / self.field.size
            shape = pick_tetrimino()

            # random coordinate to place top left of shape
            r = random.randint(0, self.num_rows - 1)
            c = random.randint(0, self.num_cols - 1)

            # places shape if it fits within field bounds
            if(can_place(shape, r, c)):
                place_shape(shape, r, c)

        self.generate_goal()
        return self.field
    
    def generate_goal(self) -> None:
        r = random.randint(self.num_rows//2 - 1, self.num_rows - 2)   # ints
        c = random.randint(self.num_cols//2 - 1, self.num_cols - 2)
        self.goal_cells = [[r, c], [r, c+1], 4]
        self.field[r, c:c+2] = 0
        self.gen_goal_region(r=r, c=c)
        print("grid goal_pos:", self.goal_cells)

    def gen_goal_region(self, r: int, c: int) -> GoalRegion:
        # two adjacent cells: (r,c) and (r,c+1)
        x0, y0 = self.cell_center_to_world(r, c)
        x1, _  = self.cell_center_to_world(r, c+1)

        half = 0.5 * self.cell_size
        margin = 0.2   # small positive buffer inside the two cells

        x_min = (min(x0, x1) - half) + margin
        x_max = (max(x0, x1) + half) - margin
        y_min = y0 - half + margin
        y_max = y0 + half - margin

        self.goal_region = GoalRegion(
            x_range=(x_min, x_max),
            y_range=(y_min, y_max),
            heading=0.0,
            heading_tol=np.deg2rad(20.0),
        )
        return self.goal_region


    """
    def display_field(self, path):
        H, W = self.field.shape

        plt.figure(figsize=(10, 10))
        plt.imshow(self.field, cmap="gray_r")
        # draw grid lines
        for row in range(H + 1):
            plt.axhline(row - 0.5, linewidth=0.5)
        for col in range(W + 1):
            plt.axvline(col - 0.5, linewidth=0.5)

        (r0, c0), (r1, c1), h = self.goal_pos
        # make sure order is top-left to bottom-right
        rmin, rmax = sorted((r0, r1))
        cmin, cmax = sorted((c0, c1))

        # if your corners are INCLUSIVE, use +1; if EXCLUSIVE, drop the +1
        height = (rmax - rmin + 1)
        width  = (cmax - cmin + 1)

        ax = plt.gca()
        rect = Rectangle(
            (cmin - 0.5, rmin - 0.5),  # (x,y) = (col,row) of top-left corner
            width, height,
            linewidth=2,
            edgecolor='orange',
            facecolor='none',         # or e.g. 'orange' with alpha=0.2 to fill
            label='Goal region')
        ax.add_patch(rect)

        if(path is not None):
            path = np.asarray(path) 
            rows, cols = path[:,0], path[:,1]
            plt.plot(cols, rows, 'b-', linewidth=2)      # blue line
            plt.plot(cols, rows, 'bo', markersize=4)     # blue dots
            plt.plot(cols[0], rows[0], 'go', markersize=8, label="Start")  # green start
            plt.plot(cols[-1], rows[-1], 'ro', markersize=8, label="Goal") # red goal

        plt.title("test")
        plt.xticks([]); plt.yticks([])
        plt.show()
    """

    def gen_motion_primitive(self,
        delta_deg: float,
        arc_l: float,
        wheelbase: float,
    )-> Tuple[Tuple[float, float], float, float]:
    
        delta = (delta_deg * math.pi) / 180.0
        t = tan(delta)
        dtheta = (arc_l/wheelbase) * t

        if isclose(t, 0.0, abs_tol=1e-12):
            x_end, y_end = arc_l, 0.0
        else:
            R = wheelbase / t
            x_end = R * np.sin(dtheta)
            y_end = R * (1.0 - np.cos(dtheta))

        return (x_end, y_end), dtheta, arc_l

    def gen_primitive_lib(self,
            steer_ang_deg: Iterable[float] = (-25.0, 0.0, 25.0),
            arc_l: Iterable[float] = (-3.0, 3.0, -1.0, 1.0),
            wheelbase: float = 2.5,
    ) -> List[Tuple[Tuple[float, float], float, float]]:
    
        lib: List[Tuple[Tuple[float, float], float, float]] = []

        for L in arc_l:
            for ang in steer_ang_deg:
                local_end, dtheta, L_out = self.gen_motion_primitive(delta_deg=ang, arc_l=L, wheelbase=wheelbase)
                # print(local_end, dtheta, L_out)
                lib.append((local_end, dtheta, L_out))

        return lib
    
    def gen_state_lattice( self,
        headings: int = 8,
        steering_angles_deg: Iterable[float] = (-45.0, -25.0, 0.0, +25.0, +45.0),
        lengths: Iterable[float] = (-3.0, 3.0, -1.0, 1.0, 1.5, -1.5, 0.5, -0.5),
        wheelbase: float = 2.5,
    ) -> Tuple[Set[Pose], Dict[Pose, List[Edge]]]:

        height, width = self.num_rows, self.num_cols

        primitive_lib = self.gen_primitive_lib(
            steer_ang_deg=steering_angles_deg,
            arc_l=lengths,
            wheelbase=wheelbase,
        )

        nodes: Set[Pose] = set()
        for iy in range(height):
            for ix in range(width):
                if self.field[iy, ix] != 0:
                    continue
                cx, cy = self.cell_center_to_world(iy, ix)
                for b in range(headings):
                    h = self.bin_to_angle(b, headings)
                    nodes.add(Pose(cx, cy, h))

        adj: Dict[Pose, List[Edge]] = {n: [] for n in nodes}

        for s in nodes:
            for endpoint_local, dtheta, arc_len in primitive_lib:
                # CHANGED: integrate the primitive you will actually collision-check
                samples = self.sample_arc_points(
                    s, dtheta=dtheta, arc_l=arc_len,
                    wheelbase=wheelbase,
                    step=self.sample_step  # CHANGED: ensure it's sample_step (singular)
                )

                # ADDED: endpoint from integration (no wx/wy rotation math)
                x_end, y_end, th_end = samples[-1]

                # ADDED (optional but robust): bounds check all samples to avoid
                # edges that “skip” outside and back in due to large steps
                out = any(
                    px < 0.0 or py < 0.0 or
                    px > self.num_cols * self.cell_size or
                    py > self.num_rows * self.cell_size
                    for (px, py, _pth) in samples
                )
                if out:
                    continue

                # CHANGED: snap endpoint position to nearest cell center
                rN, cN, xc, yc = self.world_to_nearest_cell_center(x_end, y_end)
                if self.field[rN, cN] != 0:
                    continue

                # CHANGED: wrap & bin the heading to exactly match node headings
                th_wrapped = self.wrap_angle(th_end)
                b_new = self.heading_bin(th_wrapped, headings)
                theta_q = self.bin_to_angle(b_new, headings)

                # CHANGED: re-use your edge collision checker
                if not self.is_edge_free(s, dtheta=dtheta, arc_l=arc_len, wheelbase=wheelbase):
                    continue

                tgt  = Pose(xc, yc, theta_q)
                cost = abs(arc_len) + 0.05 * abs(dtheta)

                # ADDED: keep meta so you can re-sample curved edges later
                meta = EdgeData(dtheta=dtheta, arc_len=arc_len)
                adj[s].append((tgt, cost, meta))  # CHANGED: store meta

        self.lattice = nodes, adj
        return self.lattice


    """
    def gen_state_lattice( self,                                      # CHANGED
        headings: int = 8,
        steering_angles_deg: Iterable[float] = (-25.0, 0.0, +25.0),
        lengths: Iterable[float] = (-3.0, 3.0, 0.5, -0.5, 0.25, -0.25,-1.0, 1.0),
        wheelbase: float = 2.5,
    ) -> Tuple[Set[Pose], Dict[Pose, List[Edge]]]:
        
        # Continuous-state lattice with ε-hash: nodes live at continuous (x,y,θ),
        # we dedupe/lookup using small quantization buckets (res_xy, res_th_rad).
        

        H, W = self.num_rows, self.num_cols

        # Primitive library (same as before)
        primitive_lib = self.gen_primitive_lib(
            steer_ang_deg=steering_angles_deg,
            arc_l=lengths,
            wheelbase=wheelbase,
        )

        # In-bounds check (meters)
        world_w = self.num_cols * self.cell_size
        world_h = self.num_rows * self.cell_size
        def in_world_bounds(x: float, y: float) -> bool:
            eps = 1e-9
            return (-eps <= x <= world_w + eps) and (-eps <= y <= world_h + eps)

        # --- Build nodes using ε-keys (dict) ---
        nodes_by_key: dict[tuple[int,int,int], Pose] = {}            # NEW
        adj: Dict[Pose, List[Edge]] = {}                             # NEW

        # Seed from free-cell centers (ok to seed at centers; successors stay continuous)
        for r in range(H):
            for c in range(W):
                if self.field[r, c] != 0:
                    continue
                cx, cy = self.cell_center_to_world(r, c)             # keep seeding at centers
                for b in range(headings):
                    th = 2.0 * math.pi * (b / headings)
                    u = self.get_or_make_node(cx, cy, th, nodes_by_key)  # NEW
                    if u not in adj:
                        adj[u] = []                                  # NEW
        # Expand closure under primitives with a simple frontier (BFS over the state graph)
        frontier = list(nodes_by_key.values())                      # (unchanged)
        idx = 0                                                    # (unchanged)

        # --- TERMINATION SAFEGUARDS ---                             # NEW
        MAX_NODES = 20000                                           # NEW: global node cap
        MAX_FRONTIER = 20000                                        # NEW: frontier cap
        MAX_NEW_EDGES_PER_NODE = 24                                 # NEW: per-node branching cap

        while (
            idx < len(frontier)
            and idx < MAX_FRONTIER                                  # NEW
            and len(nodes_by_key) < MAX_NODES                       # NEW
        ):
            s = frontier[idx]; idx += 1
            s_th = s.theta

            new_edges = 0                                           # NEW

            for (endpoint_local, dtheta, arc_len) in primitive_lib:
                # transform local primitive endpoint into world coords (unchanged math)
                wx =  endpoint_local[0] * math.cos(s_th) - endpoint_local[1] * math.sin(s_th)
                wy =  endpoint_local[0] * math.sin(s_th) + endpoint_local[1] * math.cos(s_th)
                x_new = s.x + wx
                y_new = s.y + wy
                th_new = self.wrap_angle(s_th + dtheta)             # (unchanged)

                # --- avoid self-loops caused by ε-bucketing ---     # NEW
                if self.pose_key_from_vals(x_new, y_new, th_new) == self.pose_key_from_vals(s.x, s.y, s.theta):
                    continue

                # bounds + collision along the arc (unchanged semantics)
                if not in_world_bounds(x_new, y_new):
                    continue
                if not self.is_edge_free(s, dtheta=dtheta, arc_l=arc_len, wheelbase=wheelbase):
                    continue

                # Get/create continuous target using ε-hash (NO snapping to cell centers)
                v = self.get_or_make_node(x_new, y_new, th_new, nodes_by_key)

                # Ensure adjacency list exists for v; expand from new nodes as well
                if v not in adj:
                    # --- respect node cap before adding/expanding --- # NEW
                    if len(nodes_by_key) >= MAX_NODES:
                        continue
                    adj[v] = []
                    if len(frontier) < MAX_FRONTIER:                 # NEW
                        frontier.append(v)

                # Cost + metadata (PRESERVED)
                cost = abs(arc_len) + 0.05 * abs(dtheta)
                meta = EdgeData(dtheta=dtheta, arc_len=arc_len)
                adj[s].append((v, cost, meta))

                new_edges += 1                                       # NEW
                if new_edges >= MAX_NEW_EDGES_PER_NODE:              # NEW
                    break


        # Export as (set, dict)
        nodes = set(nodes_by_key.values())                        # NEW
        self.lattice = nodes, adj
        return self.lattice
        """
        
    def display_field(
        self,
        path=None,                    # list[Pose] or list[(x,y)] if path_units="meters"; list[(row,col)] if "cells"
        *,
        path_units: str = "meters",   # "meters" or "cells" (required, default "meters")
        draw_nodes: bool = True,
        draw_edges: bool = True,
        max_edges_per_node: int = 3,
        node_stride: int = 1
    ):
        """
        Deterministic plot:
        - Occupancy grid in cell coordinates (top-left origin).
        - GoalRegion rectangle (meters, drawn over the grid).
        - Lattice nodes/edges (if present).
        - Path drawn exactly as provided (according to path_units).
        """
        import numpy as np
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle

        H, W = self.field.shape
        world_w = self.num_cols * self.cell_size
        world_h = self.num_rows * self.cell_size

        # ---- helpers (no guessing) ----
        def meters_to_plot(x_m: float, y_m: float) -> tuple[float, float]:
            # Cell center at (c+0.5, r+0.5) maps to (c, r)
            c = x_m / self.cell_size - 0.5
            r = y_m / self.cell_size - 0.5
            return c, r

        def cells_to_plot(row: float, col: float) -> tuple[float, float]:
            # A cell center (row, col) should plot at (col, row)
            return float(col), float(row)

        # ---- figure & base grid ----
        fig, ax = plt.subplots(figsize=(10, 10), dpi=140)

        ax.imshow(
            self.field, cmap="gray_r", origin="upper",
            extent=[-0.5, W - 0.5, H - 0.5, -0.5],  # top-left origin, cell coords
            zorder=0
        )

        # grid lines
        for r in range(H + 1):
            ax.axhline(r - 0.5, linewidth=0.5, color='k', alpha=0.15, zorder=1)
        for c in range(W + 1):
            ax.axvline(c - 0.5, linewidth=0.5, color='k', alpha=0.15, zorder=1)

        ax.set_xlim(-0.5, W - 0.5)
        ax.set_ylim(H - 0.5, -0.5)
        ax.set_aspect('equal', adjustable='box')

        # ---- goal region (meters) ----
        if getattr(self, "goal_region", None):
            gr = self.goal_region
            c0, r0 = meters_to_plot(gr.x_range[0], gr.y_range[0])
            c1, r1 = meters_to_plot(gr.x_range[1], gr.y_range[1])
            xmin, xmax = min(c0, c1), max(c0, c1)
            ymin, ymax = min(r0, r1), max(r0, r1)
            ax.add_patch(Rectangle(
                (xmin, ymin), xmax - xmin, ymax - ymin,
                linewidth=2, edgecolor='deepskyblue', facecolor='none',
                linestyle='--', zorder=3, label='GoalRegion (meters)'
            ))

        # ---- lattice (if present) ----
        if getattr(self, "lattice", None):
            try:
                nodes, adj = self.lattice
            except Exception:
                nodes, adj = set(), {}

            if draw_edges and adj:
                for i, (s, edges) in enumerate(adj.items()):
                    if i % max(1, node_stride) != 0:
                        continue
                    sx, sy = meters_to_plot(s.x, s.y)
                    for e in edges[:max_edges_per_node]:
                        tgt = e[0]            # handles (tgt, cost) or (tgt, cost, meta)
                        tx, ty = meters_to_plot(tgt.x, tgt.y)
                        ax.plot([sx, tx], [sy, ty], '-', linewidth=1,
                                color=(0.25, 0.5, 1.0, 0.35), zorder=2, clip_on=True)

            if draw_nodes and nodes:
                xs, ys = [], []
                for i, s in enumerate(nodes):
                    if i % max(1, node_stride) != 0:
                        continue
                    cx, cy = meters_to_plot(s.x, s.y)
                    xs.append(cx); ys.append(cy)
                if xs:
                    ax.scatter(xs, ys, s=10,
                            c=[(0.7, 0.9, 1.0, 0.75)], marker='o', edgecolors='none',
                            label="Lattice nodes", zorder=4, clip_on=True)

        # ---- path (draw exactly as provided) ----
        if path:
            if path_units == "meters":
                cols, rows = [], []
                for pt in path:
                    x = float(pt.x) if hasattr(pt, "x") else float(pt[0])
                    y = float(pt.y) if hasattr(pt, "y") else float(pt[1])
                    c, r = meters_to_plot(x, y)
                    cols.append(c); rows.append(r)
                ax.plot(cols, rows, 'b-', linewidth=2, label="Path (meters)", zorder=5)
                ax.plot(cols[0], rows[0], 'o', color='tab:green', markersize=7, label="Start", zorder=6)
                ax.plot(cols[-1], rows[-1], 'o', color='tab:red', markersize=7, label="Goal", zorder=6)

                # explicit debug
                print("Display -> Path units: meters | Start(m):",
                    (round(float(path[0].x if hasattr(path[0], 'x') else path[0][0]), 3),
                    round(float(path[0].y if hasattr(path[0], 'y') else path[0][1]), 3)),
                    "Goal(m):",
                    (round(float(path[-1].x if hasattr(path[-1], 'x') else path[-1][0]), 3),
                    round(float(path[-1].y if hasattr(path[-1], 'y') else path[-1][1]), 3)))

            elif path_units == "cells":
                cols = [float(rc[1]) for rc in path]
                rows = [float(rc[0]) for rc in path]
                ax.plot(cols, rows, 'b-', linewidth=2, label="Path (cells)", zorder=5)
                ax.plot(cols[0], rows[0], 'o', color='tab:green', markersize=7, label="Start", zorder=6)
                ax.plot(cols[-1], rows[-1], 'o', color='tab:red', markersize=7, label="Goal", zorder=6)

                # explicit debug
                print("Display -> Path units: cells | Start(rc):",
                    (path[0][0], path[0][1]), "Goal(rc):", (path[-1][0], path[-1][1]))
            else:
                raise ValueError("path_units must be 'meters' or 'cells'")

        # ---- final touches ----
        ax.set_title("Field + Lattice (deterministic display)")
        ax.set_xticks([]); ax.set_yticks([])
        handles, labels = ax.get_legend_handles_labels()
        if handles:
            ax.legend(loc="upper right")

        if getattr(self, "goal_region", None):
            print("Display -> GoalRegion(m):", self.goal_region.x_range, self.goal_region.y_range)

        plt.show()







"""
    def gen_state_lattice( self,
        headings: int = 25,
        steering_angles_deg: Iterable[float] = (-25.0, 0.0, +25.0),
        lengths: Iterable[float] = (-3.0, 3.0, -4.5, 4.5, -1.5, 1.5, -1.0, 1.0, 0.5, -0.5),
        wheelbase: float = 2.5,
    ) -> Tuple[Set[Pose], Dict[Pose, List[Edge]]]:

        height, width = self.num_rows, self.num_cols

        # primitive library (closed-form arcs)
        primitive_lib = self.gen_primitive_lib(
            steer_ang_deg=steering_angles_deg,
            arc_l=lengths,
            wheelbase=wheelbase,
        )


        # --- initialize a sparse, continuous node set ---
        nodes: Set[Pose] = set()

        for iy in range(height):
            for ix in range(width):
                if self.field[iy, ix] != 0:
                    continue
                # convert cell center to continuous world coordinates (m)
                # cx, cy = self.cell_center_to_world(iy, ix)
                cx, cy = ix, iy
                # single representative heading — this will expand continuously
                for b in range(headings):
                    h = self.bin_to_angle(b, headings)
                    nodes.add(Pose(cx, cy, h))

        # adjacency dictionary for edges
        adj: Dict[Pose, List[Edge]] = {n: [] for n in nodes}

        # main loop: generate continuous connections
        for s in nodes:
            for endpoint_local, dtheta, arc_len in primitive_lib:
                # theta = s.theta  # now treated as a continuous heading in radians

                samples = self.sample_arc_points(
                    s, dtheta=dtheta, arc_l=arc_len,
                    wheelbase=wheelbase,
                    step=self.sample_step  # CHANGED: ensure it's sample_step (singular)
                )

                # ADDED: endpoint from integration (no wx/wy rotation math)
                x_end, y_end, th_end = samples[-1]
                theta_new = self.wrap_angle(th_end)
                b_new = self.heading_bin(theta_new, headings)
                # theta_q = self.bin_to_angle(b_new, headings)   # <-- exact same grid as nodes
                snapped = self.snap_pose(x=x_end, y=y_end, theta=theta_new)
                xc,yc,theta_q = snapped.x, snapped.y, snapped.theta

                # snap endpoint to nearest center
                # rN, cN, xc, yc = self.world_to_nearest_cell_center(x_end, y_end)
                # if self.field[rN, cN] != 0:
                #     continue

                # xc, yc = x_end, y_end

                if not self.is_edge_free(s, dtheta=dtheta, arc_l=arc_len, wheelbase=wheelbase):
                    continue

                tgt = Pose(xc, yc, theta_q)  # use your angle snap if you like
                cost = abs(arc_len) + 0.05 * abs(dtheta)

                meta = EdgeData(dtheta=dtheta, arc_len=arc_len)
                adj[s].append((tgt, cost, meta))

        self.lattice = nodes, adj
        
        # sample_node = next(iter(self.lattice[0]))
        # print(sample_node)
        # print(len(self.lattice[1][sample_node]))
        
        return self.lattice 
        """