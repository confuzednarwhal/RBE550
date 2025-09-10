import numpy as np
import matplotlib.pyplot as plt
from typing import Dict, List
import random

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

def build_rotations(shapes: Dict[str, np.array]) -> Dict[str, np.array]:
    rotations: Dict[str, List[np.array]] = {}

    for name, shape in shapes.items():
        rotations[name] = rotate_shape(shape)

    return rotations

TETRIMINO_ROTATIONS = build_rotations(TETRIMINOS)


class map:
    def __init__(self):
        self.num_rows: int = 64
        self.num_cols: int = 64
        self.density: float = 0.2
        self.field = np.zeros((self.num_rows, self.num_cols), dtype=np.uint8)

        self.rng = np.random.default_rng() 

        self.rotations: Dict[str, List[np.array]] = TETRIMINO_ROTATIONS

    def get_map(self) -> np.array:
        return self.field

    def pick_tetrimino(self):
        rot: np.array = []
        name = self.rng.choice(list(self.rotations.keys()))
        rot = self.rng.choice(self.rotations[name])
        return rot

    # checks to see if shape fits the bounds of the field 
    def can_place(self, shape, row, col) -> bool:
        h, w = shape.shape
        H, W = self.field.shape
        return (row + h < H and col + w < W)

    # places shape in random spot in field, handles overlapping shapes
    def place_shape(self, shape, row, col):
        h, w = shape.shape
        self.field[row:row+h, col:col+w] = np.maximum(self.field[row:row+h, col:col+w], shape)
        # field[row:row+h, col:col+w] = shape

    # generates map at specific density
    def generate_map(self) -> np.array:
        curr_density = 0
        # field = np.zeros((fieldSize_row, fieldSize_col))


        # places shapes until desired density is reached
        while curr_density <= self.density:
            curr_density = np.sum(self.field) / self.field.size
            shape = self.pick_tetrimino()

            # random coordinate to place top left of shape
            r = random.randint(0, self.num_rows - 1)
            c = random.randint(0, self.num_cols - 1)

            # places shape if it fits within field bounds
            if self.can_place(shape, r, c):
                self.place_shape(shape, r, c)
        
        return self.field
    
    def update_map(self, location: np.array = []):
        if location:


        





    def display_field(self):
        H, W = self.field.shape
        plt.figure(figsize=(10, 10))
        plt.imshow(self.field, cmap="gray_r")
        # draw grid lines
        for row in range(H + 1):
            plt.axhline(row - 0.5, linewidth=0.5)
        for col in range(W + 1):
            plt.axvline(col - 0.5, linewidth=0.5)

        plt.title("test")
        plt.xticks([]); plt.yticks([])
        plt.show()