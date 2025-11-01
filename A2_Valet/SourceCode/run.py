import world
import numpy as np
import pygame


# b0 = self.heading_bin(start.theta, headings)
# start = Pose(start.x, start.y, self.bin_to_angle(b0, headings))

test = world.map()

test2 = test.generate_map()
test.generate_goal()
test.gen_state_lattice()
test.display_field(None)