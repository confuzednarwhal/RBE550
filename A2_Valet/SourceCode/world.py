from __future__ import annotations
import numpy as np
import matplotlib.pyplot as plt
from typing import Dict, List, Tuple, Iterable, Set
import random

from dataclasses import dataclass
from math import cos, sin, tan, pi, hypot, isclose

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

class Pose:
    x: int
    y: int
    head_index: int

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
        
        return self.field
    

    # not used in this game, used to initially visualize path
    # made with help from ChatGPT
    # def display_field(self, path):
    #     H, W = self.field.shape

    #     plt.figure(figsize=(10, 10))
    #     plt.imshow(self.field, cmap="gray_r")
    #     # draw grid lines
    #     for row in range(H + 1):
    #         plt.axhline(row - 0.5, linewidth=0.5)
    #     for col in range(W + 1):
    #         plt.axvline(col - 0.5, linewidth=0.5)

    #     if(path is not None):
    #         path = np.asarray(path) 
    #         rows, cols = path[:,0], path[:,1]
    #         plt.plot(cols, rows, 'b-', linewidth=2)      # blue line
    #         plt.plot(cols, rows, 'bo', markersize=4)     # blue dots
    #         plt.plot(cols[0], rows[0], 'go', markersize=8, label="Start")  # green start
    #         plt.plot(cols[-1], rows[-1], 'ro', markersize=8, label="Goal") # red goal

    #     plt.title("test")
    #     plt.xticks([]); plt.yticks([])
    #     plt.show()




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
            arc_l: Iterable[float] = (3.0, ),
            wheelbase: float = 2.5,
    ) -> List[Tuple[Tuple[float, float], float, float]]:
    
        lib = List[Tuple[Tuple[float, float], float, float]] = []

        for L in arc_l:
            for ang in steer_ang_deg:
                local_end, dtheta, L_out = self.gen_motion_primitive(delta_deg=ang, arc_l=arc_l, wheelbase=wheelbase)
                lib.append((local_end, dtheta, L_out))

        return lib


    def gen_state_lattice( self,
        headings: int = 8,
        steering_angles_deg: Iterable[float] = (-25.0, 0.0, +25.0),
        lengths: Iterable[float] = (1.0,),
        wheelbase: float = 2.5,
    ) -> Tuple[Set[Pose], Dict[Pose, List[Edge]]]:
        
        height, width = self.num_rows, self.num_cols

        nodes: Set[Pose] = set()

        # 1) Visit every grid cell by its integer indices
        for y in range(height):        # y = row index
            for x in range(width):     # x = column index
                # 2) Skip obstacles; only place nodes on free cells
                if self.field[y, x] != 0:
                    continue
                # 3) For a free cell, place one node per heading bin
                for h in range(headings):
                    nodes.add(Pose(x=x, y=y, h=h))

        # 4) Prepare an empty adjacency list (edges will be added later)
        adj: Dict[Pose, List[Edge]] = {node: [] for node in nodes}

        # 2) Primitive library (closed-form endpoints)
        primitive_lib = self.gen_primitive_lib(
            steer_ang_deg=steering_angles_deg,
            arc_l=lengths,
            wheelbase=wheelbase,
        )

        def in_bounds(x: int, y: int) -> bool:
            return 0 <= x < width and 0 <= y < height
        

        # 3) Heading / rotation helpers
        def heading_angle(hbin: int) -> float:
            return (2.0 * pi * hbin) / headings

        def rotate_local_point(px: float, py: float, theta: float) -> Tuple[float, float]:
            return (px * cos(theta) - py * sin(theta),
                    px * sin(theta) + py * cos(theta))

        def radians_to_heading_bins(dtheta: float) -> int:
            bin_angle = (2.0 * pi) / headings
            return int(round(dtheta / bin_angle))

        # 4) Endpoint & cost helpers (closed-form endpoint version)
        def primitive_endpoint_world_cell(start: Pose, endpoint_local: Tuple[float, float]) -> Tuple[int, int] | None:
            """
            Rotate/translate the CLOSED-FORM local endpoint into world coordinates,
            then snap to a grid index (gx, gy). Returns None if out of bounds.
            """
            theta = heading_angle(start.h)
            wx, wy = rotate_local_point(endpoint_local[0], endpoint_local[1], theta)
            gx = int((start.x + 0.5) + wx)
            gy = int((start.y + 0.5) + wy)
            if not in_bounds(gx, gy):
                return None
            return gx, gy

        def primitive_cost_arc_length(arc_length: float, dtheta_bins: int) -> float:
            """
            Edge cost model:
            - Base = true arc length (provided by the primitive)
            - + tiny penalty per heading-bin change (to mildly prefer straights)
            """
            return arc_length + 0.05 * abs(dtheta_bins)

        # 5) Wire up edges (no collision checks)
        for s in nodes:
            for endpoint_local, dtheta, arc_len in primitive_lib:
                dth_bins = radians_to_heading_bins(dtheta)

                cell = primitive_endpoint_world_cell(s, endpoint_local)
                if cell is None:
                    continue

                gx, gy = cell
                tgt = Pose(gx, gy, (s.h + dth_bins) % headings)

                if tgt in nodes:
                    cost = primitive_cost_arc_length(arc_len, dth_bins)
                    adj[s].append((tgt, cost))
        
        self.lattice = nodes, adj

        return self.lattice
        


    def display_field(self,
                  path=None,
                  *,
                  headings=None,              # int (only needed if you want to filter by heading)
                  h_only=None,                # show only this heading bin (e.g., 0..headings-1) or None for all
                  draw_nodes=True,
                  draw_edges=True,
                  max_edges_per_node=3,       # limit to avoid hairball
                  node_stride=1,              # draw every Nth node for speed/clarity
                  edge_alpha=0.35,
                  node_alpha=0.75,
                  node_size=10):

        H, W = self.field.shape
        nodes, adj = self.lattice[0], self.lattice[1]

        # --- base layer: grid + obstacles (your original) ---
        plt.figure(figsize=(10, 10))
        plt.imshow(self.field, cmap="gray_r", origin="upper")  # keep same orientation
        for row in range(H + 1):
            plt.axhline(row - 0.5, linewidth=0.5, color='k', alpha=0.15)
        for col in range(W + 1):
            plt.axvline(col - 0.5, linewidth=0.5, color='k', alpha=0.15)

        # --- optional: draw a planned path (your original, unchanged) ---
        if path is not None:
            path = np.asarray(path)
            rows, cols = path[:, 0], path[:, 1]
            plt.plot(cols, rows, 'b-', linewidth=2, label="Path")
            plt.plot(cols, rows, 'bo', markersize=4)
            plt.plot(cols[0], rows[0], 'go', markersize=8, label="Start")
            plt.plot(cols[-1], rows[-1], 'ro', markersize=8, label="Goal")

        # Helpers to accept either dataclass States or raw tuples
        def _xyz(s):
            # returns (x, y, h)
            if hasattr(s, "x"):
                return s.x, s.y, getattr(s, "h", 0)
            return s  # assume tuple (x,y,h)

        # --- lattice: edges first (so nodes draw on top) ---
        if draw_edges and adj is not None:
            drawn = 0
            for i, (s, edges) in enumerate(adj.items()):
                if i % max(1, node_stride) != 0:
                    continue
                sx, sy, sh = _xyz(s)
                if h_only is not None and sh != h_only:
                    continue
                cx, cy = sx + 0.5, sy + 0.5
                # draw up to N edges per node
                for j, (tgt, _cost) in enumerate(edges[:max_edges_per_node]):
                    tx, ty, _th = _xyz(tgt)
                    plt.plot([cx, tx + 0.5], [cy, ty + 0.5],
                            '-', linewidth=1, color=(0.25, 0.5, 1.0, edge_alpha))
                drawn += 1

        # --- lattice: nodes (cell centers) ---
        if draw_nodes and nodes is not None:
            xs, ys = [], []
            for i, s in enumerate(nodes):
                if i % max(1, node_stride) != 0:
                    continue
                x, y, h = _xyz(s)
                if h_only is not None and h != h_only:
                    continue
                xs.append(x + 0.5)
                ys.append(y + 0.5)
            if xs:
                plt.scatter(xs, ys, s=node_size, c=[(0.7, 0.9, 1.0, node_alpha)], marker='o', edgecolors='none', label="Lattice nodes")

        # --- final touches ---
        plt.title("Field + State Lattice")
        plt.xticks([]); plt.yticks([])
        # show legend only if we actually drew labeled items
        handles, labels = plt.gca().get_legend_handles_labels()
        if handles:
            plt.legend(loc="upper right")
        plt.show()
