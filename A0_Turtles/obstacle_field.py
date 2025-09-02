import numpy as np
import matplotlib.pyplot as plt
import random

fieldSize_row = 128
fieldSize_col = 128

# Tetriminos
I = np.array([[1, 1, 1, 1]])
O = np.array([[1, 1],
              [1, 1]])
T = np.array([[1, 1, 1],
              [0, 1, 0]])
Z = np.array([[1, 1, 0],
              [0, 1, 1]])
L = np.array([[0, 0, 1],
              [1, 1, 1]])


tetriminos = [I, O, T, Z, L]

# generates all 4 potential rotations of shape
def rotations(shape):
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

# randomly picks a tetrimino at random rotation
def pick_tetrimino():
    shape = random.choice(tetriminos)
    rotation = random.choice(rotations(shape))
    return rotation

# checks to see if shape fits the bounds of the field 
def can_place(field, shape, row, col):
    h, w = shape.shape
    H, W = field.shape
    return (row + h < H and col + w < W)

# places shape in random spot in field, handles overlapping shapes
def place_shape(field, shape, row, col):
    h, w = shape.shape
    field[row:row+h, col:col+w] = np.maximum(field[row:row+h, col:col+w], shape)
    # field[row:row+h, col:col+w] = shape

# generates map at specific density
def generate_map(density = 0.3):
    curr_density = 0
    field = np.zeros((fieldSize_row, fieldSize_col))

    # places shapes until desired density is reached
    while curr_density <= density:
        curr_density = np.sum(field) / field.size
        shape = pick_tetrimino()

        # random coordinate to place top left of shape
        r = random.randint(0, fieldSize_row - 1)
        c = random.randint(0, fieldSize_row - 1)

        # places shape if it fits within field bounds
        if can_place(field, shape, r, c):
            place_shape(field, shape, r, c)
    
    return field

# handles displaying obstacle field
def display_field(field, title="Obstacle Map"):
    H, W = field.shape
    plt.figure(figsize=(10, 10))
    plt.imshow(field, cmap="gray_r")
    # draw grid lines
    for row in range(H + 1):
        plt.axhline(row - 0.5, linewidth=0.5)
    for col in range(W + 1):
        plt.axvline(col - 0.5, linewidth=0.5)

    plt.title(title)
    plt.xticks([]); plt.yticks([])
    plt.show()


display_field(generate_map(density=0.1), title="Obstacle Map at 10% Density")
display_field(generate_map(density=0.5), title="Obstacle Map at 50% Density")
display_field(generate_map(density=0.7), title="Obstacle Map at 70% Density")
