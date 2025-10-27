import world
import numpy as np
import pygame


test = world.map()

test2 = test.generate_map()
test.gen_state_lattice()
test.display_field(test2)