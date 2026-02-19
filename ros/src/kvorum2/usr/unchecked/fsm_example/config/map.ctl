
import random

random.seed(0)

LGOAL = gdic.LEVEL_COLOR

N_OBSTACLES = 20


# place a goal
Env.SetRectangleField(50-5, 50-5, 50+5, 50+5, LGOAL, 1)

# place a goal
Env.SetRectangleField(100-5, 150-5, 100+5, 150+5, LGOAL, 2)

# place obstacles randomly
for i in range(0, N_OBSTACLES):
    x = random.randint(0, Env.DIM_X)
    y = random.randint(0, Env.DIM_Y)
    Env.SetRectangleField(x-5, y-5, x+5, y+5, gdic.LEVEL_GROUND, 200)
