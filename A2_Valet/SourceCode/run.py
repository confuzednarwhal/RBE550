# viewer.py
from world import map as World, Pose
from planner_lazy import AStarPlanner
import pygame
import numpy as np

# --- Build world + map ---
w = World()
grid = w.generate_map()                  # np.ndarray (rows, cols) with 0/1
H, W = grid.shape

# --- Pixel scaling ---
cell_px = 50                             # pixels per *cell*
px_per_meter = cell_px / w.cell_size     # pixels per meter

# --- Screen size (fix: width uses W, not H) ---
screen_w, screen_h = W * cell_px, H * cell_px

# --- Simple conversions ---
def m2px(x_m: float, y_m: float) -> tuple[int, int]:
    return int(x_m * px_per_meter), int(y_m * px_per_meter)

def cell_center_to_m(r: int, c: int) -> tuple[float, float]:
    return (c + 0.5) * w.cell_size, (r + 0.5) * w.cell_size

# --- Plan once (simple example start) ---
# You can set your own start; here we pick the first free cell near the top-left.
start_rc = None
for r in range(H):
    for c in range(W):
        if grid[r, c] == 0:
            start_rc = (r, c)
            break
    if start_rc: break

if start_rc is None:
    raise RuntimeError("No free start cell found.")

sx_m, sy_m = cell_center_to_m(*start_rc)
start = Pose(sx_m, sy_m, 0.0)

planner = AStarPlanner(world=w)
plan_e = planner.plan(start)  # expects list of Pose or (x,y) in meters

# Normalize path to list[(x,y)] meters (no angles)
def normalize_path_m(pth):
    out = []
    if not pth:
        return out
    for p in pth:
        if hasattr(p, "x"):
            out.append((float(p.x), float(p.y)))
        else:
            out.append((float(p[0]), float(p[1])))
    return out

path_m = normalize_path_m(plan_e.path)

# --- pygame drawing helpers ---
def draw_grid(screen):
    # Vertical lines
    for x in range(0, screen_w + 1, cell_px):
        pygame.draw.line(screen, (200, 200, 200), (x, 0), (x, screen_h), 1)
    # Horizontal lines
    for y in range(0, screen_h + 1, cell_px):
        pygame.draw.line(screen, (200, 200, 200), (0, y), (screen_w, y), 1)

def draw_goal_region(screen):
    if getattr(w, "goal_region", None) is None:
        return
    gr = w.goal_region
    x0, y0 = m2px(gr.x_range[0], gr.y_range[0])
    x1, y1 = m2px(gr.x_range[1], gr.y_range[1])
    gx = min(x0, x1); gy = min(y0, y1)
    gw = abs(x1 - x0); gh = abs(y1 - y0)
    pygame.draw.rect(screen, (0, 190, 255), pygame.Rect(gx, gy, gw, gh), width=2)
    # center cross
    cx_m = 0.5 * (gr.x_range[0] + gr.x_range[1])
    cy_m = 0.5 * (gr.y_range[0] + gr.y_range[1])
    cx, cy = m2px(cx_m, cy_m)
    pygame.draw.line(screen, (0, 190, 255), (cx - 6, cy), (cx + 6, cy), 2)
    pygame.draw.line(screen, (0, 190, 255), (cx, cy - 6), (cx, cy + 6), 2)

def draw_path(screen, path_meters):
    if len(path_meters) < 2:
        return
    pts = [m2px(p.x, p.y) if hasattr(p, "x") else m2px(p[0], p[1]) for p in path_meters]
    pygame.draw.lines(screen, (50, 100, 255), False, pts, 3)
    pygame.draw.circle(screen, (0, 170, 0), pts[0], 6)       # start
    pygame.draw.circle(screen, (220, 0, 0), pts[-1], 6)      # goal (path end)

def handle_display(screen):
    # base occupancy image: free=white, obstacle=black
    rgb = np.zeros((H, W, 3), dtype=np.uint8)
    rgb[grid == 0] = [255, 255, 255]
    surface = pygame.surfarray.make_surface(np.transpose(rgb, (1, 0, 2)))
    scaled_surface = pygame.transform.scale(surface, (screen_w, screen_h))
    screen.blit(scaled_surface, (0, 0))

    draw_grid(screen)
    draw_goal_region(screen)
    draw_path(screen, plan_e.path)

    pygame.display.flip()

def run():
    pygame.init()
    pygame.display.set_caption("Valet")
    screen = pygame.display.set_mode((screen_w, screen_h))
    clock = pygame.time.Clock()

    running = True
    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.KEYDOWN and e.key in (pygame.K_ESCAPE, pygame.K_q):
                running = False

        handle_display(screen)
        clock.tick(30)

    pygame.quit()

if __name__ == "__main__":
    run()






# from world import map as World, Pose
# from planner_lazy import AStarPlanner
# import pygame
# import numpy as np

# w = World()
# map = w.generate_map()

# cell_size: int = 50
# grid_h, grid_w = map.shape
# screen_h, screen_w = grid_h*cell_size, grid_h*cell_size

# def draw_grid(screen):
#     # --- Draw grid lines ---
#     # Vertical lines
#     for x in range(0, screen_w, cell_size):
#         pygame.draw.line(screen, (200, 200, 200), (x, 0), (x, screen_h), 1)
#     # Horizontal lines
#     for y in range(0, screen_h, cell_size):
#         pygame.draw.line(screen, (200, 200, 200), (0, y), (screen_w, y), 1)

# def handle_display(screen):
#     grid_h, grid_w = map.shape
#     screen_h, screen_w = grid_h * cell_size, grid_w * cell_size
    
#     # Free cells (0) -> white [255,255,255]; walls (1) -> black [0,0,0]
#     rgb = np.zeros((grid_h, grid_w, 3), dtype=np.uint8)  # start black everywhere
#     rgb[map == 0] = [255, 255, 255]                    # paint free cells white

#     surface = pygame.surfarray.make_surface(np.transpose(rgb, (1, 0, 2)))
#     scaled_surface = pygame.transform.scale(surface, (screen_w, screen_h))

#     # screen.fill((0, 0, 0))               # optional clear (scaled_surface covers it)
#     screen.blit(scaled_surface, (0, 0))    # draw at top-left
    
#     draw_grid(screen)

#     # if(end_game):
#     #     overlay = pygame.Surface((screen_w, screen_h), pygame.SRCALPHA)
#     #     overlay.fill((0, 0, 0, 160))
#     #     screen.blit(overlay, (0, 0))
#     #     # draw_text(screen, game_state)

#     pygame.display.flip() 

# def run():
#     pygame.init()
#     pygame.font.init()

#     screen = pygame.display.set_mode((screen_w,screen_h))
#     pygame.display.set_caption("Valet")

#     clock = pygame.time.Clock()
#     running = True
#     step = 0
#     old_step = 0

#     while running:

#         handle_display(screen)
#         pygame.time.wait(2000)

# run()
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
