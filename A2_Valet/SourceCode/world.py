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
        self.density: float = 0.15
        self.field = np.zeros((self.num_rows, self.num_cols), dtype=np.uint8)
        self.rng = np.random.default_rng() 
        self.rotations: Dict[str, List[np.array]] = TETRIMINO_ROTATIONS

        self.lattice: Tuple[Set[Pose], Dict[Pose, List[Edge]]] = []
        self.goal_cells: Tuple[Tuple[int,int], Tuple[int,int], int] = []
        self.goal_region: GoalRegion | None = None

        self.vehicle_w: float = 3.0        
        self.vehicle_l: float = 6.0       
        self.obstacle_clearance: float = 0.05

      
        self.sample_step: float = 0.5          
        self.use_circle_footprint: bool = True   

    def get_map(self) -> np.array:
        return self.field
    
    def snap_pose(self, x: float, y: float, theta: float) -> Pose:
        t = (theta % (2 * pi))
        return Pose(round(x, 3), round(y, 3), round(t, 3))
    
    def world_to_cell(self, x: float, y:float) -> Tuple[int,int]:
        r = int(y // self.cell_size)
        c = int(x // self.cell_size)
        return r, c

    def is_occupied(self, r: int, c: int) -> bool:
        H, W = self.field.shape
        return r == 0 or r >= H or c == 0 or c >= W
    
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
            cx = (cc) * self.cell_size
            cy = (rr) * self.cell_size
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
    

    def sample_arc_points(self, s: Pose, dtheta: float, arc_l: float, steer_deg: float,
                           wheelbase: float, step: float) -> List[Tuple[float, float, float]]:  # ADDED
        """
        Integrate along the primitive from state s at small steps.
        Returns a list of (x, y, theta) samples including the endpoint.
        """
        import math
        n = max(2, int(abs(arc_l) / max(1e-6, step)))
        delta = math.radians(steer_deg)
        k = math.tan(delta) / wheelbase  # curvature
        x, y, th = s.x, s.y, s.theta  # using your continuous heading

        pts = []
        for _ in range(n):
            ds = arc_l / n
            if abs(k) < 1e-10:
                # straight integration
                x += ds * math.cos(th)
                y += ds * math.sin(th)
            else:
                # integrate with small rotation increment
                th += ds * k
                x += ds * math.cos(th)
                y += ds * math.sin(th)
            pts.append((x, y, th))
        return pts

    def is_edge_free(self, s: Pose,
                     endpoint_local: Tuple[float, float], dtheta: float, arc_l: float,
                     steer_deg: float, wheelbase: float) -> bool:  # ADDED

        # 1) sample along the arc in world coordinates
        samples = self.sample_arc_points(
            s, dtheta = dtheta, arc_l=arc_l,
            steer_deg = steer_deg, wheelbase=wheelbase,
            step = self.sample_step,
        )

        diag = math.hypot(self.vehicle_l, self.vehicle_w)
        radius = 0.5 * diag + self.obstacle_clearance
        for (x, y, _th) in samples:
            if self.occupied_in_radius(x, y, radius):
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
        r = random.randint(self.num_rows / 2 - 1, self.num_rows-2)
        c = random.randint(self.num_cols / 2 -1, self.num_cols-2)
        self.goal_cells = [[r,c], [r,c+1], 4]
        self.field[r, c:c+2] = 0

        self.gen_goal_region(r=r, c=c)

        print("grid goal_pos:", self.goal_cells)

    def gen_goal_region(self, r: int, c: int) -> GoalRegion:
        x0_center_m = (c) * self.cell_size     # left cell center X
        x1_center_m = (c+1) * self.cell_size # right cell center X
        y_center_m  = (r) * self.cell_size

        # Parking box (world meters): a rectangle covering both cells with a small margin
        margin = -1.0  # meters (small cushion)
        x_min = x0_center_m - (self.cell_size / 2) + margin
        x_max = x1_center_m + (self.cell_size / 2) - margin
        y_min = y_center_m  - (self.cell_size / 2) + margin
        y_max = y_center_m  + (self.cell_size / 2) - margin

        # Desired heading and tolerance
        desired_heading = 0.0
        heading_tol = np.deg2rad(20.0)

        self.goal_region = GoalRegion(  
            x_range=(x_min, x_max),
            y_range=(y_min, y_max),
            heading=desired_heading,
            heading_tol=heading_tol
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

        def rotate_local_point(px: float, py: float, theta: float) -> Tuple[float, float]:
            return (
                px * cos(theta) + py * sin(theta),
                -px * sin(theta) + py * cos(theta)
            )

        def in_world_bounds(x: float, y: float) -> bool:
            eps = 1e-6                               # FIX: small tolerance
            world_w = self.num_cols * self.cell_size
            world_h = self.num_rows * self.cell_size
            return (-eps <= x <= world_w + eps) and (-eps <= y <= world_h + eps) 

        def primitive_cost(arc_length: float, dtheta: float) -> float:
            return abs(arc_length) + 0.05 * abs(dtheta)

        # --- initialize a sparse, continuous node set ---
        nodes: Set[Pose] = set()

        for iy in range(height):
            for ix in range(width):
                if self.field[iy, ix] != 0:
                    continue
                # convert cell center to continuous world coordinates (m)
                cx = (ix) * self.cell_size
                cy = (iy) * self.cell_size
                # single representative heading — this will expand continuously
                for h in np.linspace(0, 2*pi, num=8, endpoint=False):
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
                theta_new = (theta + dtheta) % (2*pi)

                if not (in_world_bounds(x=x_new,y=y_new)):  # CHANGED
                    continue

                # ADDED: collision gate – reject edges whose *arc* intersects obstacles
                steer_deg = np.degrees(tan(dtheta * (self.vehicle_l / arc_len))) if not isclose(arc_len, 0.0) else 0.0  # ADDED (best-effort steer estimate for logging; not used)
                # Use the same steer you used to make (endpoint_local, dtheta, arc_len) if you kept it; 
                # If not, you can ignore 'steer_deg' here and pass a nominal value (0°) because _sample_arc_points uses dtheta & arc_l.

                if not self.is_edge_free(s,
                                         endpoint_local=endpoint_local,
                                         dtheta=dtheta,
                                         arc_l=arc_len,
                                         steer_deg=0.0,          # ADDED: we don't actually need steer here (sampler uses dtheta & arc_l)
                                         wheelbase=wheelbase):
                    continue

                # ADDED: snap the new pose so dict/set keys remain stable
                tgt = self.snap_pose(x_new, y_new, theta_new)  # ADDED

                # CHANGED: cost unchanged; add if free
                cost = abs(arc_len) + 0.05 * abs(dtheta)  # same as before
                adj[s].append((tgt, cost))  # CHANGED: 'tgt' is snapped Pose

        self.lattice = nodes, adj
        return self.lattice
    
        # n_edges = sum(len(e) for e in adj.values())
        # n_with_edges = sum(1 for e in adj.values() if e)
        # print("edges:", n_edges, "nodes_with_edges:", n_with_edges, "of", len(nodes))
        
    # def display_field(self,
    #     path=None,
    #     *,
    #     headings=None,              # (optional) number of bins, only used if you filter by bin
    #     h_only=None,                # EITHER an int bin (0..headings-1) OR a tuple (angle_rad, tol_rad)
    #     draw_nodes=True,
    #     draw_edges=True,
    #     max_edges_per_node=3,
    #     node_stride=1,
    #     edge_alpha=0.35,
    #     node_alpha=0.75,
    #     node_size=10):

    #     """
    #     Visualizes:
    #     - occupancy grid (in cell coordinates)
    #     - a continuous lattice whose node coords are in METERS (x[m], y[m], theta[rad])
    #     - optional path (assumed in cell coords, same as your original)
    #     """

    #     import math
    #     H, W = self.field.shape
    #     nodes, adj = self.lattice

    #     # ---------- helpers ----------
    #     def m_to_cells(v_m: float) -> float:
    #         """Convert meters to cell units for plotting over the grid image."""
    #         return v_m / self.cell_size

    #     def to_plot_xy(state):
    #         """Return (col, row) in grid/cell units from a Pose or (x,y,θ)."""
    #         if hasattr(state, "x"):
    #             x_m = state.x
    #             y_m = state.y
    #         else:
    #             x_m, y_m = state[0], state[1]
    #         return m_to_cells(x_m), m_to_cells(y_m)

    #     def heading_of(state) -> float:
    #         """Return heading in radians from a Pose or 3-tuple."""
    #         if hasattr(state, "theta"):
    #             return state.theta  # in your continuous lattice this is θ (rad)
    #         return state[2]

    #     def angle_diff(a: float, b: float) -> float:
    #         """Smallest signed difference a-b in [-pi, pi]."""
    #         d = (a - b + math.pi) % (2 * math.pi) - math.pi
    #         return d

    #     def heading_bin(theta: float, n_bins: int) -> int:
    #         """Map a continuous angle to the nearest discrete heading bin."""
    #         step = 2.0 * math.pi / n_bins
    #         return int(round((theta % (2*math.pi)) / step)) % n_bins

    #     def pass_heading_filter(theta: float) -> bool:
    #         """Implements h_only semantics for continuous headings."""
    #         if h_only is None:
    #             return True
    #         # Case 1: user passed an integer bin and provided 'headings'
    #         if isinstance(h_only, int) and isinstance(headings, int) and headings > 0:
    #             return heading_bin(theta, headings) == h_only
    #         # Case 2: user passed (angle_rad, tol_rad)
    #         if isinstance(h_only, (tuple, list)) and len(h_only) == 2:
    #             ang, tol = h_only
    #             return abs(angle_diff(theta, ang)) <= tol
    #         # Fallback: show all
    #         return True

    #     # ---------- base layer: grid + obstacles ----------
    #     plt.figure(figsize=(10, 10), dpi=140)
    #     plt.imshow(self.field, cmap="gray_r", origin="upper", zorder=0)
    #     for row in range(H + 1):
    #         plt.axhline(row - 0.5, linewidth=0.5, color='k', alpha=0.15, zorder=1)
    #     for col in range(W + 1):
    #         plt.axvline(col - 0.5, linewidth=0.5, color='k', alpha=0.15, zorder=1)

    #     # ---------- optional: draw a planned path (assumed in CELL coords, like your original) ----------
    #     if path is not None:
    #         path = np.asarray(path)
    #         rows, cols = path[:, 0], path[:, 1]
    #         plt.plot(cols, rows, 'b-', linewidth=2, label="Path", zorder=5)
    #         plt.plot(cols, rows, 'bo', markersize=4, zorder=5)
    #         plt.plot(cols[0], rows[0], 'go', markersize=8, label="Start", zorder=6)
    #         plt.plot(cols[-1], rows[-1], 'ro', markersize=8, label="Goal", zorder=6)

    #     # ---------- lattice: edges first (nodes on top) ----------
    #     if draw_edges and adj:
    #         for i, (s, edges) in enumerate(adj.items()):
    #             if i % max(1, node_stride) != 0:
    #                 continue
    #             sh = heading_of(s)
    #             if not pass_heading_filter(sh):
    #                 continue
    #             sx_c, sy_c = to_plot_xy(s)  # already in cell units
    #             # draw up to N edges per node
    #             for tgt, _cost in edges[:max_edges_per_node]:
    #                 tx_c, ty_c = to_plot_xy(tgt)
    #                 plt.plot([sx_c, tx_c], [sy_c, ty_c],
    #                         '-', linewidth=1,
    #                         color=(0.25, 0.5, 1.0, edge_alpha),
    #                         zorder=3)

    #     # ---------- lattice: nodes ----------
    #     if draw_nodes and nodes:
    #         xs, ys = [], []
    #         for i, s in enumerate(nodes):
    #             if i % max(1, node_stride) != 0:
    #                 continue
    #             sh = heading_of(s)
    #             if not pass_heading_filter(sh):
    #                 continue
    #             x_c, y_c = to_plot_xy(s)
    #             xs.append(x_c)
    #             ys.append(y_c)
    #         if xs:
    #             plt.scatter(xs, ys,
    #                         s=node_size,
    #                         c=[(0.7, 0.9, 1.0, node_alpha)],
    #                         marker='o',
    #                         edgecolors='none',
    #                         label="Lattice nodes",
    #                         zorder=4)

    #     # ---------- final touches ----------
    #     plt.title("Field + Continuous-State Lattice")
    #     plt.xticks([]); plt.yticks([])
    #     handles, labels = plt.gca().get_legend_handles_labels()
    #     if handles:
    #         plt.legend(loc="upper right")
    #     plt.show()

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

        # --- world (meters) -> plot coordinates (cells) over a top-origin image ---
        def world_to_cell_plot(x_m: float, y_m: float) -> tuple[float, float]:
            c = x_m / self.cell_size
            r = (world_h - y_m) / self.cell_size if y_up_world else y_m / self.cell_size
            return c, r

        fig, ax = plt.subplots(figsize=(10, 10), dpi=140)

        # 1) Put the image in the SAME coordinate system as the overlays
        #    extent sets the image coords to [-0.5..W-0.5] x [-0.5..H-0.5] with top-origin.
        ax.imshow(
            self.field, cmap="gray_r", origin="upper",
            extent=[-0.5, W - 0.5, H - 0.5, -0.5],  # <-- key: coordinates in cell units
            zorder=0
        )

        # 2) Nice grid lines aligned with cell edges
        for r in range(H + 1):
            ax.axhline(r - 0.5, linewidth=0.5, color='k', alpha=0.15, zorder=1)
        for c in range(W + 1):
            ax.axvline(c - 0.5, linewidth=0.5, color='k', alpha=0.15, zorder=1)

        # 3) Keep plotting inside the board & preserve square cells
        ax.set_xlim(-0.5, W - 0.5)
        ax.set_ylim(H - 0.5, -0.5)  # top-origin
        ax.set_aspect('equal', adjustable='box')  # <-- avoids distortion

        # --- draw metric GoalRegion (optional) ---
        if getattr(self, "goal_region", None):
            gr = self.goal_region
            c0, r0 = world_to_cell_plot(gr.x_range[0], gr.y_range[0])
            c1, r1 = world_to_cell_plot(gr.x_range[1], gr.y_range[1])
            xmin, xmax = min(c0, c1), max(c0, c1)
            ymin, ymax = min(r0, r1), max(r0, r1)
            ax.add_patch(Rectangle((xmin, ymin),
                                xmax - xmin, ymax - ymin,
                                linewidth=2, edgecolor='deepskyblue',
                                facecolor='none', linestyle='--', zorder=3,
                                label='Parking (meters)'))

        # --- draw cell-space goal_pos (optional) ---
        if getattr(self, "goal_cells", None):
            (r0, c0), (r1, c1), _ = self.goal_cells
            rmin, rmax = sorted((r0, r1))
            cmin, cmax = sorted((c0, c1))
            ax.add_patch(Rectangle((cmin - 0.5, rmin - 0.5),
                                (cmax - cmin + 1), (rmax - rmin + 1),
                                linewidth=2, edgecolor='green',
                                facecolor='none', zorder=3,
                                label='Parking (cells)'))

        # --- draw path in cell coords (rows, cols) ---
        if path is not None:
            p = np.asarray(path)
            rows, cols = p[:, 0], p[:, 1]
            ax.plot(cols, rows, 'b-', linewidth=2, label="Path", zorder=5)
            ax.plot(cols, rows, 'bo', markersize=4, zorder=5)
            ax.plot(cols[0], rows[0], 'go', markersize=8, label="Start", zorder=6)
            ax.plot(cols[-1], rows[-1], 'ro', markersize=8, label="Goal", zorder=6)

        # --- draw lattice (world meters -> cells) ---
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
                    for tgt, _ in edges[:max_edges_per_node]:
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
                    xs.append(cx); ys.append(cy)
                if xs:
                    ax.scatter(xs, ys,
                            s=node_size,
                            c=[(0.7, 0.9, 1.0, node_alpha)],
                            marker='o', edgecolors='none',
                            label="Lattice nodes", zorder=4, clip_on=True)

        ax.set_title("Field + Continuous-State Lattice")
        ax.set_xticks([]); ax.set_yticks([])
        handles, labels = ax.get_legend_handles_labels()
        if handles:
            ax.legend(loc="upper right")
        plt.show()
