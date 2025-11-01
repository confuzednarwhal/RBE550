from __future__ import annotations
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from typing import Dict, List, Tuple, Iterable, Set
import random
import math

from dataclasses import dataclass
from math import cos, sin, tan, pi, hypot, isclose, floor

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

@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    theta: float 

@dataclass()
class GoalRegion:
    x_range: Tuple[float,float]
    y_range: Tuple[float,float]
    heading: float
    heading_tol: float

Edge = Tuple[Pose, float]



class map:
    def __init__(self):
        self.cell_size: float = 3.0
        self.num_rows: int = 12
        self.num_cols: int = 12
        self.density: float = 0.05
        self.field = np.zeros((self.num_rows, self.num_cols), dtype=np.uint8)
        self.rng = np.random.default_rng() 
        self.rotations: Dict[str, List[np.array]] = TETRIMINO_ROTATIONS

        self.lattice: Tuple[Set[Pose], Dict[Pose, List[Edge]]] = []
        self.goal_cells: Tuple[Tuple[int,int], Tuple[int,int], int] = []
        self.goal_region: GoalRegion | None = None

        self.vehicle_w: float = 3.0        
        self.vehicle_l: float = 6.0       
        self.obstacle_clearance: float = 2.0

      
        self.sample_step: float = 0.25          
        self.use_circle_footprint: bool = True   

    def get_map(self) -> np.array:
        return self.field
    
    def wrap_angle(self, th: float) -> float:
        return th % (2 * math.pi)

    def heading_bin(self, th: float, N: int) -> int:
        # map angle to nearest bin index 0..N-1
        th = self.wrap_angle(th)
        step = 2 * math.pi / N
        return int(round(th / step)) % N

    def bin_to_angle(self, th: float, N: int) -> float:
        step = 2 * math.pi / N
        b = int(round(self.wrap_angle(th) / step)) % N
        return b * step 
    
    def snap_pose(self, x: float, y: float, theta: float) -> Pose:
        t = (theta % (2 * pi))
        return Pose(round(x, 3), round(y, 3), round(t, 3))
    
    def world_to_cell(self, x: float, y:float) -> Tuple[int,int]:
        r = int(y // self.cell_size)
        c = int(x // self.cell_size)
        return r, c

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

    def is_occupied(self, r: int, c: int) -> bool:
        H, W = self.field.shape
        return (r < 0) or (r >= H) or (c < 0) or (c >= W)
    
    def occupied_in_radius(self, x: float, y: float, radius_m: float) -> bool: 
        """Return True if any *in-bounds* obstacle cell center lies within radius_m of (x,y)."""
        H, W = self.field.shape
        r0, c0 = self.world_to_cell(x, y)

        # search window in cell indices, clipped to the grid
        k = int(np.ceil(radius_m / self.cell_size))
        rmin = max(0, r0 - k); rmax = min(H - 1, r0 + k)
        cmin = max(0, c0 - k); cmax = min(W - 1, c0 + k)

        # helper: cell center in meters, honoring your y convention
        world_h = self.num_rows * self.cell_size
        def cell_center_m(rr: int, cc: int) -> tuple[float, float]:
            cx = (cc + 0.5) * self.cell_size
            cy = (rr + 0.5) * self.cell_size
            return cx, cy

        r2 = radius_m * radius_m
        for rr in range(rmin, rmax + 1):
            for cc in range(cmin, cmax + 1):
                if self.field[rr, cc] == 0:
                    continue  # free cell
                cx, cy = cell_center_m(rr, cc)
                if (cx - x) * (cx - x) + (cy - y) * (cy - y) <= r2:
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
    
        delta = (delta_deg * pi) / 180.0
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
        steering_angles_deg: Iterable[float] = (-25.0, 0.0, +25.0),
        lengths: Iterable[float] = (-3.0, 3.0, -1.0, 1.0),
        wheelbase: float = 2.5,
    ) -> Tuple[Set[Pose], Dict[Pose, List[Edge]]]:
        """
        Generate a continuous state lattice — no grid snapping.
        Nodes live in (x [m], y [m], theta [rad]).
        The occupancy grid is only used later for collision queries.
        """

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
                cx, cy = self.cell_center_to_world(iy, ix)
                # single representative heading — this will expand continuously
                for b in range(headings):
                    h = self.bin_to_angle(b, headings)
                    nodes.add(Pose(cx, cy, h))

        # adjacency dictionary for edges
        adj: Dict[Pose, List[Edge]] = {n: [] for n in nodes}

        # main loop: generate continuous connections
        for s in nodes:
            for endpoint_local, dtheta, arc_len in primitive_lib:
                theta = s.theta  # now treated as a continuous heading in radians

                # transform primitive endpoint into world space
                wx, wy = (
                    endpoint_local[0] * cos(theta) - endpoint_local[1] * sin(theta),
                    endpoint_local[0] * sin(theta) + endpoint_local[1] * cos(theta))
                
                x_new = s.x + wx
                y_new = s.y + wy 
                theta_new = self.wrap_angle(s.theta + dtheta)
                b_new = self.heading_bin(theta_new, headings)
                theta_q = self.bin_to_angle(b_new, headings)   # <-- exact same grid as nodes
                

                # snap endpoint to nearest center
                rN, cN, xc, yc = self.world_to_nearest_cell_center(x_new, y_new)
                # reject if that center’s cell is occupied
                if self.field[rN, cN] != 0:
                    continue

                if not self.is_edge_free(s, dtheta=dtheta, arc_l=arc_len, wheelbase=wheelbase):
                    continue

                tgt = Pose(xc, yc, theta_q)  # use your angle snap if you like
                cost = abs(arc_len) + 0.05 * abs(dtheta)
                adj[s].append((tgt, cost))

        self.lattice = nodes, adj
        
        # sample_node = next(iter(self.lattice[0]))
        # print(sample_node)
        # print(len(self.lattice[1][sample_node]))
        
        return self.lattice
    
    def display_field(self,
        path=None,
        *,
        y_up_world: bool = False,      # True if your lattice world is y-up; False if y-down
        draw_nodes: bool = True,
        draw_edges: bool = True,
        max_edges_per_node: int = 3,
        node_stride: int = 1,
        edge_alpha: float = 0.35,
        node_alpha: float = 0.75,
        node_size: int = 10):

        import numpy as np
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle

        H, W = self.field.shape
        world_w = self.num_cols * self.cell_size
        world_h = self.num_rows * self.cell_size

        # --- world (meters) -> plot coordinates (cells) ---
        # FIX: subtract 0.5 so that cell centers map to integer coordinates
        #      when drawn over imshow(extent=[-0.5, W-0.5, H-0.5, -0.5])
        def world_to_cell_plot(x_m: float, y_m: float) -> tuple[float, float]:
            c = x_m / self.cell_size - 0.5          # FIX
            if y_up_world:
                r = (world_h - y_m) / self.cell_size - 0.5   # FIX
            else:
                r = y_m / self.cell_size - 0.5               # FIX
            return c, r

        fig, ax = plt.subplots(figsize=(10, 10), dpi=140)

        # --- display occupancy grid ---
        ax.imshow(
            self.field, cmap="gray_r", origin="upper",
            extent=[-0.5, W - 0.5, H - 0.5, -0.5],  # top-left corner of top-left cell is (-0.5,-0.5)
            zorder=0
        )

        # --- grid lines (aligned to cell edges) ---
        for r in range(H + 1):
            ax.axhline(r - 0.5, linewidth=0.5, color='k', alpha=0.15, zorder=1)
        for c in range(W + 1):
            ax.axvline(c - 0.5, linewidth=0.5, color='k', alpha=0.15, zorder=1)

        # --- keep square cells ---
        ax.set_xlim(-0.5, W - 0.5)
        ax.set_ylim(H - 0.5, -0.5)  # top-origin
        ax.set_aspect('equal', adjustable='box')

        # --- draw metric GoalRegion ---
        if getattr(self, "goal_region", None):
            gr = self.goal_region
            c0, r0 = world_to_cell_plot(gr.x_range[0], gr.y_range[0])
            c1, r1 = world_to_cell_plot(gr.x_range[1], gr.y_range[1])
            xmin, xmax = min(c0, c1), max(c0, c1)
            ymin, ymax = min(r0, r1), max(r0, r1)
            ax.add_patch(Rectangle(
                (xmin, ymin),
                xmax - xmin, ymax - ymin,
                linewidth=2, edgecolor='deepskyblue',
                facecolor='none', linestyle='--', zorder=3,
                label='Parking (meters)'
            ))

        # --- draw discrete goal cells (optional) ---
        if getattr(self, "goal_cells", None):
            (r0, c0), (r1, c1), _ = self.goal_cells
            rmin, rmax = sorted((r0, r1))
            cmin, cmax = sorted((c0, c1))
            ax.add_patch(Rectangle(
                (cmin - 0.5, rmin - 0.5),
                (cmax - cmin + 1), (rmax - rmin + 1),
                linewidth=2, edgecolor='green',
                facecolor='none', zorder=3,
                label='Parking (cells)'
            ))

        # --- draw path (in cell coordinates) ---
        if path is not None:
            p = np.asarray(path)
            rows, cols = p[:, 0], p[:, 1]
            ax.plot(cols, rows, 'b-', linewidth=2, label="Path", zorder=5)
            ax.plot(cols, rows, 'bo', markersize=4, zorder=5)
            ax.plot(cols[0], rows[0], 'go', markersize=8, label="Start", zorder=6)
            ax.plot(cols[-1], rows[-1], 'ro', markersize=8, label="Goal", zorder=6)

        # --- draw lattice edges and nodes ---
        if getattr(self, "lattice", None):
            try:
                nodes, adj = self.lattice
            except Exception:
                nodes, adj = set(), {}

            if draw_edges and adj:
                for i, (s, edges) in enumerate(adj.items()):
                    if i % max(1, node_stride) != 0:
                        continue
                    sx, sy = world_to_cell_plot(s.x, s.y)
                    limit = len(edges) if max_edges_per_node is None else max_edges_per_node
                    for tgt, _ in edges[:limit]:
                        tx, ty = world_to_cell_plot(tgt.x, tgt.y)
                        ax.plot([sx, tx], [sy, ty],
                                '-', linewidth=1,
                                color=(0.25, 0.5, 1.0, edge_alpha),
                                zorder=2, clip_on=True)

            if draw_nodes and nodes:
                xs, ys = [], []
                for i, s in enumerate(nodes):
                    if i % max(1, node_stride) != 0:
                        continue
                    cx, cy = world_to_cell_plot(s.x, s.y)
                    xs.append(cx)
                    ys.append(cy)
                if xs:
                    ax.scatter(xs, ys,
                            s=node_size,
                            c=[(0.7, 0.9, 1.0, node_alpha)],
                            marker='o', edgecolors='none',
                            label="Lattice nodes", zorder=4, clip_on=True)

        ax.set_title("Field + Continuous-State Lattice (cell-centered)")
        ax.set_xticks([]); ax.set_yticks([])
        handles, labels = ax.get_legend_handles_labels()
        if handles:
            ax.legend(loc="upper right")
        plt.show()
