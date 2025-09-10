import numpy as np
from typing import Dict, List

class entity:
    def __init__(self):
        self.pos: np.array = []
        self.color: str = ""
        self.goal: np.array = []
        self.alive: bool = True

    def get_pos(self) -> np.array:
        return self.pos
    
    def update_pos(self):
        self.pos = [0, 0]

    def place_entity(self):

        return 0
    
    def gen_trajectory(self):
        return 0
    
class hero(entity):
    def __init__(self):
        self.danger: bool = False  # enemy close
        self.color = "blue"

    def enemy_prox(self, pos: np.array) -> bool:
        # calculate enemy prox to hero
        return 0


class enemy(entity):
    def __init__(self):
        self.color = "red"
        self.obstacle: bool = False  #even necessary?


