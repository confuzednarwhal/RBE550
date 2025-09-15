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

cell_size: int = 15
grid_h, grid_w = field.shape
screen_h, screen_w = grid_h*cell_size, grid_h*cell_size

# Used ChatGPT to help with display and its helpers
def draw_grid(screen):
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

def draw_hero(screen, coords):
    coord_x, coord_y = get_cell_center(coords[0], coords[1])
    coord_r = max(2, cell_size // 2 - 1)
    pygame.draw.circle(screen, hero1.color, (coord_x, coord_y), coord_r)

def draw_goal(screen, coords):
    goal_color = (0, 255, 0)
    # coord_x, coord_y = get_cell_center(coords[0], coords[1], cell_size)
    coord_x, coord_y = coords[1]*cell_size, coords[0]*cell_size
    rect = pygame.Rect(coord_x, coord_y, cell_size, cell_size)
    pygame.draw.rect(screen, goal_color, rect)

def draw_enemy(screen, enemy, coords):
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

def draw_text(screen, text: str):
    font = pygame.font.SysFont(None, 64)
    text_surf = font.render(text, True, (255, 255, 255))
    rect = text_surf.get_rect(center = (screen.get_width()//2, screen.get_height()//2))
    screen.blit(text_surf, rect) 

def handle_display(screen, end_game: bool, game_state: str):
    grid_h, grid_w = field.shape
    screen_h, screen_w = grid_h * cell_size, grid_w * cell_size
    
    # Free cells (0) -> white [255,255,255]; walls (1) -> black [0,0,0]
    rgb = np.zeros((grid_h, grid_w, 3), dtype=np.uint8)  # start black everywhere
    rgb[field == 0] = [255, 255, 255]                    # paint free cells white

    surface = pygame.surfarray.make_surface(np.transpose(rgb, (1, 0, 2)))
    scaled_surface = pygame.transform.scale(surface, (screen_w, screen_h))

    # screen.fill((0, 0, 0))               # optional clear (scaled_surface covers it)
    screen.blit(scaled_surface, (0, 0))    # draw at top-left
    
    draw_grid(screen)
    #draw hero
    draw_hero(screen, hero1.get_pos())
    # draw goal point
    draw_goal(screen, hero1.get_goal())
    # draw_hero(cell_size, screen, hero1.get_goal())
    for e in enemies:
        if(e is not None):
            # print("alive:", e.alive, "pos:", e.get_pos())
            draw_enemy(screen, e, e.get_pos())

    if(end_game):
        overlay = pygame.Surface((screen_w, screen_h), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 160))
        screen.blit(overlay, (0, 0))
        draw_text(screen, game_state)

    pygame.display.flip() 


def run():
    pygame.init()
    pygame.font.init()

    screen = pygame.display.set_mode((screen_w,screen_h))
    pygame.display.set_caption("Flatlanders")

    end_game = False  # flag to kill the game
    game_state: str = "" # message to display at the end game

    # init hero and enemy goal, start, and enemy positions
    hero1.gen_goal(field)
    hero1.place_hero(field, new_pos=None)

    enemy_positions: list[tuple[int,int]] = []
    enemy_layer: np.array

    for e in enemies:
        e.place_enemy(field, new_pos = None)
        enemy_positions.append(e.get_pos())

    clock = pygame.time.Clock()
    running = True
    step = 0
    old_step = 0

    # create the first cost layer for hero pathing
    enemy_layer: np.array = base_entity.gen_cost_layer(field, enemy_positions)

    while running:
        # for easy window killing
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # teleports hero if possible every __ ticks
        if(step - old_step > 20):
            hero1.place_hero(field, new_pos=None)
            old_step = step

        # recalculates enemy cost layer for hero pathing
        # path plan for hero
        enemy_layer = base_entity.gen_cost_layer(field, enemy_positions)
        hero_path = hero1.gen_path(field, enemy_layer)

        # fix to stop enemy from suffocating in walls but not turning into debris
        # as well as index issues
        next_enemy_goal: tuple[int,int]

        if not hero_path or len(hero_path) < 2:
            next_enemy_goal = hero1.get_pos()
        else:
            next_enemy_goal = hero_path[1]

        # handles all enemy placement and hero killing logic
        # index is for rewriting the enemy position list for hero pathing cost stuff
        enemy_index = 0
        for e in enemies:
            e_path: np.array[int] = []

            # kill hero if enemy reached it
            if(e.at_goal()):
                hero1.alive = False
                break
            
            # if enemy exists, is alive, and hero is alive: set its goal and gen path
            if(e.alive and e.pos is not None and hero1.alive):
                e.set_goal(next_enemy_goal)
                e_path = e.gen_path(field)
            # iterates enemy through its path if hero isnt dead
            if (not e.at_goal() and e_path):
                # if cell is empty, proceeed with moving into it
                if (e.alive and field[e_path[1]] == 0):
                    e.place_enemy(field, new_pos = e_path[1])
                    enemy_positions[enemy_index]=(e.get_pos())
                    # print("here")
                # if cell isnt empty, make debris and die
                elif (e.alive and field[e_path[1]] == 1):
                    field[e_path[0]] = 1
                    e.alive = False

            enemy_index+=1
            

        """TODO Dynamic stuff here"""
        # handles moving the hero through its path
        # ends the game if hero is dead
        if(not hero1.at_goal() and hero1.alive):
            hero1.place_hero(field, new_pos=hero_path[1])
        else:
            end_game = True
            if(hero1.at_goal()):
                game_state = "Hero has escaped!"
            else:
                game_state = "Hero is dead :("

            handle_display(screen, end_game, game_state)
            pygame.time.wait(2000)
            break

        handle_display(screen, end_game, game_state)
        # limit fps
        clock.tick(6) 
        # interate though path
        step += 1


    # pygame.time.wait(2000)
    pygame.quit()


run()