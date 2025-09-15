import numpy as np
import math
import heapq
from typing import Dict, List
import random

from scipy.ndimage import gaussian_filter

class entity:
    def __init__(self):
        self.color: tuple[int,int,int]
        self.pos: tuple[int,int] = (0,0)
        self.goal: tuple[int,int] = (0,1)
        self.alive: bool = True
        self.moves = [(1,0),( -1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]

    def get_pos(self) -> tuple[int,int]:
        return self.pos
    
    def at_goal(self) -> bool:
        return self.goal == self.pos
    
    def update_pos(self, position: tuple[int,int]):
        self.pos = position

    def pick_pos(self, field: np.array) -> tuple[int,int]:
        try_place = True
        num_rows, num_cols = field.shape
        while try_place:
            r = random.randint(0, num_rows - 1)
            c = random.randint(0, num_cols - 1)
            new_pos = (r,c)
            if(field[new_pos] == 0):
                # try_place = False
                # return new_pos
                break
        return new_pos
    
    def gen_cost_layer(self, field: np.array, enemy_pos: list[tuple[int,int]]) -> np.array:
        free_mask = (field == 0).astype(float)
        sigma: float = 2.0
        weight: float = 500.0
        ceiling = 100000

        impulses = np.zeros(field.shape, dtype = float)
        for row, col in enemy_pos:
            if 0 <= row < field.shape[0] and 0 <= col < field.shape[1]:
                impulses[row,col] += 1.0

        layer = gaussian_filter(impulses, sigma=sigma, mode="nearest")
        layer *= weight
        layer *= free_mask
        return np.clip(layer, 0.0, ceiling)

    
class hero(entity):

    def __init__(self):
        super().__init__()
        # self.danger: bool = False  # enemy close
        self.color = (50, 160, 255)
        self.reset: int = 0
        self.teleport_lim: int = 4

    def can_teleport(self) -> bool:
        return self.reset < self.teleport_lim
    
    def get_goal(self) -> tuple[int,int]:
        return self.goal

    def gen_goal(self, field: np.array):
        self.goal = self.pick_pos(field)

    def place_hero(self, field: np.array, new_pos: tuple = None):
        if new_pos is None:
            # Teleport only while under the limit
            if self.can_teleport():
                self.pos = self.pick_pos(field)
                self.reset += 1
            # else: do nothing this tick (no teleport), but DO NOT block normal moves elsewhere
            return
        self.update_pos(new_pos)

    def gen_path(self, field: np.array, cell_cost: np.ndarray) -> np.array:
        field_row, field_cols = field.shape
        blocked = field !=0

        sr, sc = self.pos
        gr, gc = self.goal
        if blocked[sr, sc] or blocked[gr, gc]:
            # print("path blocked")
            return None
        if self.pos == self.goal:
            return [self.pos]

        def penalty_at(row: int, col: int) -> float:
            if (not cell_cost is None):
                return float(cell_cost[row, col])
            else:
                return 0
    
        def heuristic(a, b):
            # Octile distance: good for 8-direction grids
            dx = abs(a[0] - b[0])
            dy = abs(a[1] - b[1])
            return dx + dy + (math.sqrt(2) - 2) * min(dx, dy)

        def move_cost(dr, dc): #TODO: Add enemy avoidance here
            #Straight = 1, diagonal = sqrt(2)
            if(dr != 0 and dc != 0):
                return math.sqrt(2) 
            else:
                return 1.0
        
        def in_bounds(r, c):
            return 0 <= r < field_row and 0 <= c < field_cols
            
        g = {self.pos: 0.0}
        came_from: dict = {}
        open_set = [(heuristic(self.pos, self.goal), sr, sc)]
        closed_set = set()

        while(open_set):
            _, r, c = heapq.heappop(open_set)
            if (r, c) in closed_set:
                continue
            closed_set.add((r, c))
            # If we reached the goal, reconstruct and return the path
            if (r, c) == self.goal:
                path = [(r, c)]
                # Walk backwards from goal to start using came_from
                while (r, c) != self.pos:
                    r, c = came_from[(r, c)]
                    path.append((r, c))
                # Reverse to get start -> goal order
                return path[::-1]
        

            # Otherwise, consider all valid neighbors
            for dr, dc in self.moves:
                nr, nc = r + dr, c + dc

                # Skip neighbors that are out of bounds or blocked
                if not in_bounds(nr, nc) or blocked[nr, nc]:
                    continue

                # Skip if we already finalized this neighbor
                if (nr, nc) in closed_set:
                    continue

                # calculate costs
                base_cost = move_cost(dr,dc)
                enemy_cost = (penalty_at(r,c) + penalty_at(nr, nc))/2 # average enemy penalty with neighbor
                
                # Calculate the tentative new g-cost via current cell
                tentative_g = g[(r, c)] + move_cost(dr, dc) + base_cost + enemy_cost

                # If this path to (nr, nc) is better than any previously found, record it
                if tentative_g < g.get((nr, nc), float('inf')):
                    # print("huh")
                    g[(nr, nc)] = tentative_g
                    came_from[(nr, nc)] = (r, c)

                    # Compute f = g + h for priority queue ordering
                    f = tentative_g + heuristic((nr, nc), self.goal)

                    heapq.heappush(open_set, (f, nr, nc))

        # If we empty the open set without reaching the goal, no path exists
        return None



class enemy(entity):
    def __init__(self):
        super().__init__()
        self.color = (255, 0, 0)

    def set_goal(self, hero_pos):
        self.goal = hero_pos

    def place_enemy(self, field: np.array, new_pos: tuple = None):
        if(new_pos is None):
            self.update_pos(self.pick_pos(field))
        elif(not self.alive):
            return 0
        else:
            self.update_pos(new_pos)

    def gen_path(self, field: np.array) -> np.array:
        field_row, field_cols = field.shape
        blocked = field !=0

        sr, sc = self.pos
        gr, gc = self.goal
        if blocked[sr, sc] or blocked[gr, gc]:
            print("path blocked")
            self.alive = False
            return None
        if self.pos == self.goal:
            return [self.pos]
    
        def heuristic(a, b):
            # Octile distance: good for 8-direction grids
            dx = abs(a[0] - b[0])
            dy = abs(a[1] - b[1])
            return dx + dy + (math.sqrt(2) - 2) * min(dx, dy)

        def move_cost(dr, dc): #TODO: Add enemy avoidance here
            #Straight = 1, diagonal = sqrt(2)
            if(dr != 0 and dc != 0):
                return math.sqrt(2) 
            else:
                return 1.0
        
        def in_bounds(r, c):
            return 0 <= r < field_row and 0 <= c < field_cols
            

        g = {self.pos: 0.0}
        came_from = {}
        open_set = [(heuristic(self.pos, self.goal), sr, sc)]
        closed_set = set()

        while(open_set):
            _, r, c = heapq.heappop(open_set)
            if (r, c) in closed_set:
                continue
            closed_set.add((r, c))
            # If we reached the goal, reconstruct and return the path
            if (r, c) == self.goal:
                path = [(r, c)]
                # Walk backwards from goal to start using came_from
                while (r, c) != self.pos:
                    r, c = came_from[(r, c)]
                    path.append((r, c))
                    # Reverse to get start -> goal order

                return path[::-1]
        

            # Otherwise, consider all valid neighbors
            for dr, dc in self.moves:
                nr, nc = r + dr, c + dc

                # Skip neighbors that are out of bounds or blocked
                if not in_bounds(nr, nc): # or blocked[nr, nc]:
                    continue

                # Skip if we already finalized this neighbor
                if (nr, nc) in closed_set:
                    continue

                # Calculate the tentative new g-cost via current cell
                tentative_g = g[(r, c)] + move_cost(dr, dc)

                # If this path to (nr, nc) is better than any previously found, record it
                if tentative_g < g.get((nr, nc), float('inf')):
                    # print("huh")
                    g[(nr, nc)] = tentative_g
                    came_from[(nr, nc)] = (r, c)

                    # Compute f = g + h for priority queue ordering
                    f = tentative_g + heuristic((nr, nc), self.goal)
                    heapq.heappush(open_set, (f, nr, nc))
