import env_handler as Enviornment
import entity
import numpy as np
import pygame

test = Enviornment.map()
base_entity = entity.entity()
hero1 = entity.hero()
enemy1 = entity.enemy()
field = test.generate_map()

enemies = [entity.enemy() for _ in range(10)]   


cell_size: int = 10
grid_h, grid_w = field.shape
screen_h, screen_w = grid_h*cell_size, grid_h*cell_size

screen = pygame.display.set_mode((screen_w,screen_h))
surface = pygame.surfarray.make_surface(field)

def draw_grid():
    # --- Draw grid lines ---
    # Vertical lines
    for x in range(0, screen_w, cell_size):
        pygame.draw.line(screen, (200, 200, 200), (x, 0), (x, screen_h), 1)
    # Horizontal lines
    for y in range(0, screen_h, cell_size):
        pygame.draw.line(screen, (200, 200, 200), (0, y), (screen_w, y), 1)

def get_cell_center(row: int, col: int) -> tuple[int,int]:
    x = col * cell_size + cell_size // 2
    y = row * cell_size + cell_size // 2
    return (x, y)

def draw_hero(coords):
    coord_x, coord_y = get_cell_center(coords[0], coords[1])
    coord_r = max(2, cell_size // 2 - 1)
    pygame.draw.circle(screen, hero1.color, (coord_x, coord_y), coord_r)

def draw_goal(coords):
    goal_color = (0, 255, 0)
    # coord_x, coord_y = get_cell_center(coords[0], coords[1], cell_size)
    coord_x, coord_y = coords[1]*cell_size, coords[0]*cell_size
    rect = pygame.Rect(coord_x, coord_y, cell_size, cell_size)
    pygame.draw.rect(screen, goal_color, rect)

def draw_enemy(enemy, coords):
    if(not enemy.alive):
        return
    if(coords is None):
        return
    
    coord_x, coord_y = coords[1]*cell_size, coords[0]*cell_size 

    size_ratio = int(cell_size * 0.8)

    offset_x = coord_x + (cell_size - size_ratio) // 2
    offset_y = coord_y + (cell_size - size_ratio) // 2

    # print("bruh")

    tri = [(offset_x + size_ratio//2, offset_y), (offset_x, offset_y + size_ratio), (offset_x + size_ratio, offset_y + size_ratio)]
    pygame.draw.polygon(screen, enemy.color, tri)
    

def handle_display():
    grid_h, grid_w = field.shape
    screen_h, screen_w = grid_h * cell_size, grid_w * cell_size
    
    """Convert map to rbg colors and display it"""
    # Free cells (0) -> white [255,255,255]; walls (1) -> black [0,0,0]
    rgb = np.zeros((grid_h, grid_w, 3), dtype=np.uint8)  # start black everywhere
    rgb[field == 0] = [255, 255, 255]                    # paint free cells white

    surface = pygame.surfarray.make_surface(np.transpose(rgb, (1, 0, 2)))
    scaled_surface = pygame.transform.scale(surface, (screen_w, screen_h))

    # screen.fill((0, 0, 0))               # optional clear (scaled_surface covers it)
    screen.blit(scaled_surface, (0, 0))    # draw at top-left
    
    draw_grid()
    #draw hero
    draw_hero(hero1.get_pos())
    # draw goal point
    draw_goal(hero1.get_goal())
    # draw_hero(cell_size, screen, hero1.get_goal())
    for e in enemies:
        if(e is not None):
            # print("alive:", e.alive, "pos:", e.get_pos())
            draw_enemy(e, e.get_pos())

    pygame.display.flip() 


def run():
    pygame.init()

    # field = test.generate_map()
    hero1.gen_goal(field)
    hero1.place_hero(field, new_pos=None)

    enemy_positions: list[tuple[int,int]] = []
    enemy_layer: np.array

    for e in enemies:
        e.pos = e.place_enemy(field, new_pos = None)
        pos = e.get_pos()
        if pos is not None:
            enemy_positions.append(e.get_pos())

    # start_e = enemy1.place_enemy(field, new_pos = None)

    # hero_path: list = hero1.gen_path(field)
    # hero_path_size: int = len(hero_path)
    # hero_next: int = 0
    # enemy_next: int = 0

    """
    cell_size: int = 10
    grid_h, grid_w = field.shape
    screen_h, screen_w = grid_h*cell_size, grid_h*cell_size

    screen = pygame.display.set_mode((screen_w,screen_h))
    surface = pygame.surfarray.make_surface(field)
    """

    clock = pygame.time.Clock()
    running = True
    step = 0
    old_step = 0

    # enemy_positions = [enemy1.get_pos()]
    enemy_layer: np.array = base_entity.gen_cost_layer(field, enemy_positions)

    while running:

        if(step - old_step > 10):
            hero1.place_hero(field, new_pos=None)
            old_step = step


        # enemy_positions: list = [enemy1.get_pos()]
        enemy_layer = base_entity.gen_cost_layer(field, enemy_positions)
        hero_path = hero1.gen_path(field, enemy_layer)

        for e in enemies:
            e_path: np.array[int] = []
            if(e.alive and e.pos is not None):
                e.set_goal(hero1.get_pos())
                e_path = e.gen_path(field)

            if (not e.at_goal() and e_path):
                if (e.alive and field[e_path[1]] == 0):
                    e.place_enemy(field, new_pose = e_path[1])
                elif (e.alive and field[e_path[1]] != 1):
                    field[e_path[0]] = 1
                    e.alive = False
                    # e.place_enemy(field, new_pose = e_path[1])
            

        """TODO Dynamic stuff here"""
        if(not hero1.at_goal() and not enemy1.at_goal()):
            hero1.place_hero(field, new_pos=hero_path[1])

        else:
            break

        handle_display()

        """
        #Convert map to rbg colors and display it
        # Free cells (0) -> white [255,255,255]; walls (1) -> black [0,0,0]
        rgb = np.zeros((grid_h, grid_w, 3), dtype=np.uint8)  # start black everywhere
        rgb[field == 0] = [255, 255, 255]                    # paint free cells white

        surface = pygame.surfarray.make_surface(np.transpose(rgb, (1, 0, 2)))
        scaled_surface = pygame.transform.scale(surface, (screen_w, screen_h))

        # screen.fill((0, 0, 0))               # optional clear (scaled_surface covers it)
        screen.blit(scaled_surface, (0, 0))    # draw at top-left
        
        draw_grid(screen_w, screen_h, cell_size, screen)
        #draw hero
        draw_hero(cell_size, screen, hero1.get_pos())
        # draw goal point
        draw_goal(cell_size, screen, hero1.get_goal())
        # draw_hero(cell_size, screen, hero1.get_goal())
        draw_enemy(cell_size, screen, enemy1.get_pos())
        
        pygame.display.flip()
        """

        # limit fps
        clock.tick(10) 

        # interate though path
        step += 1

    pygame.quit()




run()