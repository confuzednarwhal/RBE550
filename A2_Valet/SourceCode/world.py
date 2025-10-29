from __future__ import annotations
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from typing import Dict, List, Tuple, Iterable, Set
import random

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
    rotations = []
    isDuplicate = False
    curr = shape

    for i in range(3):
        # does not generate a rotation if there is a duplicate
        for r in rotations:
            isDuplicate = np.array_equal(curr, r)
    
        if not isDuplicate:
            rotations.append(curr)
            curr = np.rot90(curr)

    return rotations

# builds dictionary of the rotations once, so no need to recompute each time
def build_rotations(shapes: Dict[str, np.array]) -> Dict[str, np.array]:
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
    head_index: int 

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
        self.goal_pos: Tuple[Tuple[int,int], Tuple[int,int], int] = []

    def get_map(self) -> np.array:
        return self.field

    # generates map at specific density
    def generate_map(self) -> np.array:
        curr_density = 0
        # field = np.zeros((fieldSize_row, fieldSize_col))
    
    
        # picks a random rotated tetrimino
        def pick_tetrimino():
            rot: np.array = []
            name = self.rng.choice(list(self.rotations.keys()))
            rot = self.rng.choice(self.rotations[name])
            return rot

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
        r = random.randint(self.num_rows/2 - 1 , self.num_rows - 1)
        c = random.randint(self.num_cols/2 - 1, self.num_cols - 1)

        self.goal_pos = [[r,c], [r,c+1], 4]
        self.field[r, c:c+2] = 0
        print(self.goal_pos)

    def gen_goal_region(self) -> GoalRegion:
        x_region: Tuple = []

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
            arc_l: Iterable[float] = (-3.0, 3.0),
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
        lengths: Iterable[float] = (-3.0,3.0),
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

        # --- helper: rotate & translate local endpoints ---
        def rotate_local_point(px: float, py: float, theta: float) -> Tuple[float, float]:
            return (
                px * cos(theta) - py * sin(theta),
                px * sin(theta) + py * cos(theta)
            )

        def in_world_bounds(x: float, y: float) -> bool:
            """Check if a world coordinate (meters) lies within map limits."""
            world_w = width * self.cell_size
            world_h = height * self.cell_size
            return (0.0 <= x < world_w) and (0.0 <= y < world_h)

        def primitive_cost(arc_length: float, dtheta: float) -> float:
            """Cost = arc length + small curvature penalty."""
            return abs(arc_length) + 0.05 * abs(dtheta)

        # --- initialize a sparse, continuous node set ---
        nodes: Set[Pose] = set()

        # Option 1: seed from free-cell centers (converted to meters)
        for iy in range(height):
            for ix in range(width):
                if self.field[iy, ix] != 0:
                    continue
                # convert cell center to continuous world coordinates (m)
                cx = (ix + 0.5) * self.cell_size
                cy = (iy + 0.5) * self.cell_size
                # single representative heading — this will expand continuously
                for h in np.linspace(0, 2*pi, num=8, endpoint=False):
                    nodes.add(Pose(cx, cy, h))

        # adjacency dictionary for edges
        adj: Dict[Pose, List[Edge]] = {n: [] for n in nodes}

        # --- main loop: generate continuous connections ---
        for s in nodes:
            for endpoint_local, dtheta, arc_len in primitive_lib:
                theta = s.head_index  # now treated as a continuous heading in radians

                # transform primitive endpoint into world space
                wx, wy = rotate_local_point(endpoint_local[0], endpoint_local[1], theta)
                x_new = s.x + wx
                y_new = s.y + wy
                theta_new = (theta + dtheta) % (2*pi)

                # skip if the target is outside the map
                if not in_world_bounds(x_new, y_new):
                    continue

                tgt = Pose(x_new, y_new, theta_new)

                # cost purely from primitive geometry
                cost = primitive_cost(arc_len, dtheta)

                # add edge
                adj[s].append((tgt, cost))

        self.lattice = nodes, adj
        return self.lattice
    
        # n_edges = sum(len(e) for e in adj.values())
        # n_with_edges = sum(1 for e in adj.values() if e)
        # print("edges:", n_edges, "nodes_with_edges:", n_with_edges, "of", len(nodes))
        
    def display_field(self,
        path=None,
        *,
        headings=None,              # (optional) number of bins, only used if you filter by bin
        h_only=None,                # EITHER an int bin (0..headings-1) OR a tuple (angle_rad, tol_rad)
        draw_nodes=True,
        draw_edges=True,
        max_edges_per_node=3,
        node_stride=1,
        edge_alpha=0.35,
        node_alpha=0.75,
        node_size=10):

        """
        Visualizes:
        - occupancy grid (in cell coordinates)
        - a continuous lattice whose node coords are in METERS (x[m], y[m], theta[rad])
        - optional path (assumed in cell coords, same as your original)
        """

        import math
        H, W = self.field.shape
        nodes, adj = self.lattice

        # ---------- helpers ----------
        def m_to_cells(v_m: float) -> float:
            """Convert meters to cell units for plotting over the grid image."""
            return v_m / self.cell_size

        def to_plot_xy(state):
            """Return (col, row) in grid/cell units from a Pose or (x,y,θ)."""
            if hasattr(state, "x"):
                x_m = state.x
                y_m = state.y
            else:
                x_m, y_m = state[0], state[1]
            return m_to_cells(x_m), m_to_cells(y_m)

        def heading_of(state) -> float:
            """Return heading in radians from a Pose or 3-tuple."""
            if hasattr(state, "head_index"):
                return state.head_index  # in your continuous lattice this is θ (rad)
            return state[2]

        def angle_diff(a: float, b: float) -> float:
            """Smallest signed difference a-b in [-pi, pi]."""
            d = (a - b + math.pi) % (2 * math.pi) - math.pi
            return d

        def heading_bin(theta: float, n_bins: int) -> int:
            """Map a continuous angle to the nearest discrete heading bin."""
            step = 2.0 * math.pi / n_bins
            return int(round((theta % (2*math.pi)) / step)) % n_bins

        def pass_heading_filter(theta: float) -> bool:
            """Implements h_only semantics for continuous headings."""
            if h_only is None:
                return True
            # Case 1: user passed an integer bin and provided 'headings'
            if isinstance(h_only, int) and isinstance(headings, int) and headings > 0:
                return heading_bin(theta, headings) == h_only
            # Case 2: user passed (angle_rad, tol_rad)
            if isinstance(h_only, (tuple, list)) and len(h_only) == 2:
                ang, tol = h_only
                return abs(angle_diff(theta, ang)) <= tol
            # Fallback: show all
            return True

        # ---------- base layer: grid + obstacles ----------
        plt.figure(figsize=(10, 10), dpi=140)
        plt.imshow(self.field, cmap="gray_r", origin="upper", zorder=0)
        for row in range(H + 1):
            plt.axhline(row - 0.5, linewidth=0.5, color='k', alpha=0.15, zorder=1)
        for col in range(W + 1):
            plt.axvline(col - 0.5, linewidth=0.5, color='k', alpha=0.15, zorder=1)

        # ---------- optional: draw a planned path (assumed in CELL coords, like your original) ----------
        if path is not None:
            path = np.asarray(path)
            rows, cols = path[:, 0], path[:, 1]
            plt.plot(cols, rows, 'b-', linewidth=2, label="Path", zorder=5)
            plt.plot(cols, rows, 'bo', markersize=4, zorder=5)
            plt.plot(cols[0], rows[0], 'go', markersize=8, label="Start", zorder=6)
            plt.plot(cols[-1], rows[-1], 'ro', markersize=8, label="Goal", zorder=6)

        # ---------- lattice: edges first (nodes on top) ----------
        if draw_edges and adj:
            for i, (s, edges) in enumerate(adj.items()):
                if i % max(1, node_stride) != 0:
                    continue
                sh = heading_of(s)
                if not pass_heading_filter(sh):
                    continue
                sx_c, sy_c = to_plot_xy(s)  # already in cell units
                # draw up to N edges per node
                for tgt, _cost in edges[:max_edges_per_node]:
                    tx_c, ty_c = to_plot_xy(tgt)
                    plt.plot([sx_c, tx_c], [sy_c, ty_c],
                            '-', linewidth=1,
                            color=(0.25, 0.5, 1.0, edge_alpha),
                            zorder=3)

        # ---------- lattice: nodes ----------
        if draw_nodes and nodes:
            xs, ys = [], []
            for i, s in enumerate(nodes):
                if i % max(1, node_stride) != 0:
                    continue
                sh = heading_of(s)
                if not pass_heading_filter(sh):
                    continue
                x_c, y_c = to_plot_xy(s)
                xs.append(x_c)
                ys.append(y_c)
            if xs:
                plt.scatter(xs, ys,
                            s=node_size,
                            c=[(0.7, 0.9, 1.0, node_alpha)],
                            marker='o',
                            edgecolors='none',
                            label="Lattice nodes",
                            zorder=4)

        # ---------- final touches ----------
        plt.title("Field + Continuous-State Lattice")
        plt.xticks([]); plt.yticks([])
        handles, labels = plt.gca().get_legend_handles_labels()
        if handles:
            plt.legend(loc="upper right")
        plt.show()

