import numpy as np
import math
import heapq
from typing import Dict, List, Tuple
import random
from world import Pose, GoalRegion

from scipy.ndimage import gaussian_filter


class vehicle:
    def __init__(self):
        self.pos: Tuple[float, float, int] = [1.5, 6.0, 4]
        self.goal: Tuple[float, float, int] = []
        self.cell_size: float = 3.0
        self.at_goal = False
    
    def set_goal(self, goal: Tuple[float, float, int]) -> None:
        x, y, h = goal[0]*self.cell_size,goal[1]*self.cell_size, goal[3]
        self.goal = [x,y,h]

    def collision_check(self) -> bool:
        return True
    
    def hybrid_a(self, start: Pose, goal_region: GoalRegion, field: np.array):
        

        return

    