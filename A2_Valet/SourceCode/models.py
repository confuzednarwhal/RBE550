import numpy as np
import math
import heapq
from typing import Dict, List, Tuple
import random

from scipy.ndimage import gaussian_filter


class vehicle:
    def __init__(self):
        self.pos: Tuple[float, float, int] = [1.5, 6.0, 4]
        self.goal: Tuple[float, float, int] = []
        self.cell_size: float = 3.0
    
    def set_goal(self, goal: Tuple[float, float, int]):
        