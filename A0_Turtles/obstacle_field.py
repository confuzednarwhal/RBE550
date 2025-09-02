import numpy as np
import matplotlib.pyplot as plt
from collections import deque
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
        for r in rotations:
            isDuplicate = np.array_equal(curr, r)
    
        if not isDuplicate:
            rotations.append()
            curr = np.rot90

    return 0

def pick_tetrimino():
    return 0

def generate_map (density):
    return 0