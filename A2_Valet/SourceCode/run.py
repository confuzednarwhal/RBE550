from world import map as World, Pose
from planner_lazy import AStarPlanner

w = World()
w.generate_map()

# w.generate_goal()
# w.gen_state_lattice()  # optional now; you don’t need the big closure anymore

# pick a start pose (meters)
start = Pose(x=1.5, y=4.5, theta=0.0)  # cell center of (0,0) for cell_size=3.0

planner = AStarPlanner(world=w)

result = planner.plan(start)
print("success:", result.success, "expanded:", result.expanded, "cost:", result.cost)

# visualize if you like
if result.success:
    w.display_field(path=result.path, path_units="meters")
