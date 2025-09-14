import env_handler as Enviornment
import entity
import numpy as np
import pygame

test = Enviornment.map()
base_entity = entity.entity()
hero1 = entity.hero()
enemy1 = entity.enemy()
field = test.generate_map()

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

def draw_enemy(coords):
    if(enemy1.alive):
        coord_x, coord_y = coords[1]*cell_size, coords[0]*cell_size 

        size_ratio = int(cell_size * 0.8)

        offset_x = coord_x + (cell_size - size_ratio) // 2
        offset_y = coord_y + (cell_size - size_ratio) // 2

        # print("bruh")

        tri = [(offset_x + size_ratio//2, offset_y), (offset_x, offset_y + size_ratio), (offset_x + size_ratio, offset_y + size_ratio)]
        pygame.draw.polygon(screen, enemy1.color, tri)
    

def handle_display():
    grid_h, grid_w = field.shape
    screen_h, screen_w = grid_h*cell_size, grid_h*cell_size
    
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
    draw_enemy(enemy1.get_pos())

    pygame.display.flip() 


def run():
    pygame.init()

    # field = test.generate_map()
    goal = hero1.gen_goal(field)
    start = hero1.place_hero(field, new_pos=None)

    start_e = enemy1.place_enemy(field, new_pos = None)

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
    # enemy_layer = base_entity.gen_cost_layer(field, enemy_positions)

    while running:

        if(step - old_step > 20):
            hero1.place_hero(field, new_pos=None)

            old_step = step
            # hero1.increment_reset()
            print(hero1.reset)


        enemy_positions: list = [enemy1.get_pos()]
        enemy_layer = base_entity.gen_cost_layer(field, enemy_positions)
        hero_path: list = hero1.gen_path(field, enemy_layer)

        if(enemy1.alive):
            enemy1.set_goal(hero1.get_pos())
            enemy_path = enemy1.gen_path(field)

        """TODO Dynamic stuff here"""
        if(not hero1.at_goal() and not enemy1.at_goal()):

            hero1.place_hero(field, new_pos=hero_path[1])

            if(enemy1.alive and field[enemy_path[1]] == 0):
                enemy1.place_enemy(field,new_pos=enemy_path[1])
            elif(enemy1.alive and field[enemy_path[1]] != 0):
                field[enemy_path[0]] = 1
                enemy1.alive = False
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
        # hero_next += 1
        step += 1

    pygame.quit()

# print(enemy1.get_pos())
# print(hero1.get_pos())

run()