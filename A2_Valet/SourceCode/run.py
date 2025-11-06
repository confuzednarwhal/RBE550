from world import map as World, Pose
from planner_lazy import AStarPlanner
import pygame
import numpy as np

w = World()
map = w.generate_map()

cell_size: int = 50
grid_h, grid_w = map.shape
screen_h, screen_w = grid_h*cell_size, grid_h*cell_size

def draw_grid(screen):
    # --- Draw grid lines ---
    # Vertical lines
    for x in range(0, screen_w, cell_size):
        pygame.draw.line(screen, (200, 200, 200), (x, 0), (x, screen_h), 1)
    # Horizontal lines
    for y in range(0, screen_h, cell_size):
        pygame.draw.line(screen, (200, 200, 200), (0, y), (screen_w, y), 1)

def handle_display(screen):
    grid_h, grid_w = map.shape
    screen_h, screen_w = grid_h * cell_size, grid_w * cell_size
    
    # Free cells (0) -> white [255,255,255]; walls (1) -> black [0,0,0]
    rgb = np.zeros((grid_h, grid_w, 3), dtype=np.uint8)  # start black everywhere
    rgb[map == 0] = [255, 255, 255]                    # paint free cells white

    surface = pygame.surfarray.make_surface(np.transpose(rgb, (1, 0, 2)))
    scaled_surface = pygame.transform.scale(surface, (screen_w, screen_h))

    # screen.fill((0, 0, 0))               # optional clear (scaled_surface covers it)
    screen.blit(scaled_surface, (0, 0))    # draw at top-left
    
    draw_grid(screen)

    # if(end_game):
    #     overlay = pygame.Surface((screen_w, screen_h), pygame.SRCALPHA)
    #     overlay.fill((0, 0, 0, 160))
    #     screen.blit(overlay, (0, 0))
    #     # draw_text(screen, game_state)

    pygame.display.flip() 

def run():
    pygame.init()
    pygame.font.init()

    screen = pygame.display.set_mode((screen_w,screen_h))
    pygame.display.set_caption("Valet")

    clock = pygame.time.Clock()
    running = True
    step = 0
    old_step = 0

    while running:

        handle_display(screen)
        pygame.time.wait(2000)

run()
"""

# pick a start pose (meters)
start = Pose(x=1.5, y=4.5, theta=0.0)  # cell center of (0,0) for cell_size=3.0

planner = AStarPlanner(world=w)

result = planner.plan(start)
print("success:", result.success, "expanded:", result.expanded, "cost:", result.cost)

# visualize if you like
if result.success:
    w.display_field(path=result.path, path_units="meters")
"""
