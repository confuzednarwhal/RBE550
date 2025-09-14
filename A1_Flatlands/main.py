import env_handler as Enviornment
import entity
import numpy as np
import pygame


def draw_grid(screen_w, screen_h, cell_size, screen):
    # --- Draw grid lines ---
    # Vertical lines
    for x in range(0, screen_w, cell_size):
        pygame.draw.line(screen, (200, 200, 200), (x, 0), (x, screen_h), 1)
    # Horizontal lines
    for y in range(0, screen_h, cell_size):
        pygame.draw.line(screen, (200, 200, 200), (0, y), (screen_w, y), 1)

def get_cell_center(row: int, col: int, cell_size: int) -> tuple[int,int]:
    x = col * cell_size + cell_size // 2
    y = row * cell_size + cell_size // 2
    return (x, y)

def draw(cell_size: int, screen, coord: tuple[int,int], color: tuple[int,int,int]):
    coord_x, coord_y = get_cell_center(coord[0], coord[1], cell_size)
    coord_r = max(2, cell_size // 2 - 1)
    pygame.draw.circle(screen, color, (coord_x, coord_y), coord_r)


def run():
    pygame.init()

    hero_next: int = 0

    cell_size = 10
    grid_h, grid_w = field.shape
    screen_h, screen_w = grid_h*cell_size, grid_h*cell_size

    screen = pygame.display.set_mode((screen_w,screen_h))
    surface = pygame.surfarray.make_surface(field)

    clock = pygame.time.Clock()
    running = True
    step = 0

    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False

        """TODO Dynamic stuff here"""
        if(not hero1.at_goal()):
            hero1.place_hero(field, new_pos=hero_path[hero_next])
            hero_next += 1
            step += 1
        elif(hero1.at_goal):
            break

        """Convert map to rbg colors and display it"""
        # Free cells (0) -> white [255,255,255]; walls (1) -> black [0,0,0]
        rgb = np.zeros((grid_h, grid_w, 3), dtype=np.uint8)  # start black everywhere
        rgb[field == 0] = [255, 255, 255]                    # paint free cells white


        surface = pygame.surfarray.make_surface(np.transpose(rgb, (1, 0, 2)))
        scaled_surface = pygame.transform.scale(surface, (screen_w, screen_h))

        # screen.fill((0, 0, 0))               # optional clear (scaled_surface covers it)
        screen.blit(scaled_surface, (0, 0))    # draw at top-left
        draw_grid(screen_w, screen_h, cell_size, screen)

        # draw hero
        draw(cell_size, screen, hero1.get_pos(), color=hero1.color)
        # draw goal point
        draw(cell_size, screen, hero1.get_goal(), color=(0, 255, 0))
        
        pygame.display.flip()

        # limit fps
        clock.tick(4) 

        # interate though path

    pygame.quit()

    

test = Enviornment.map()
hero1 = entity.hero()

field = test.generate_map()
goal = hero1.gen_goal(field)
start = hero1.place_hero(field, new_pos=None)

hero_path: list = hero1.gen_path(field)
hero_path_size: int = len(hero_path)

run()