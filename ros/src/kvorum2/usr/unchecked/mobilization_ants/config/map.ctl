
import random

random.seed(0)

LFOOD = gdic.LEVEL_COLOR
LDANGER = gdic.LEVEL_LIGHT

N_OBSTACLES = 50

def CField(x, y, dim):
    global LFOOD, LDANGER
    Env.SetRoundField(x, y, dim, LDANGER, 1)
    Env.SetRoundField(x, y, dim-5, LFOOD, 7)

def FoodLightField(x, y, dim):
    global LFOOD, LDANGER
    x1 = int(x-dim/2)
    y1 = int(y-dim/2)
    x2 = int(x+dim/2)
    y2 = int(y+dim/2)
    Env.SetRectangleField(x1, y1, x2, y2, LFOOD, 7)
    Env.SetRectangleField(x1-5, y1-5, x2+5, y2+5, LDANGER, 200)

def FoodLightObstField(x, y, dim):
    global LFOOD, LDANGER
    x1 = int(x-dim/2)
    y1 = int(y-dim/2)
    x2 = int(x+dim/2)
    y2 = int(y+dim/2)

    Env.SetRectangleField(x1,    y1,    x2,    y2,    gdic.LEVEL_GROUND, 200)    
    Env.SetRectangleField(x1,    y1,    x2,    y2,    LFOOD,    0)    
    Env.SetRectangleField(x1-5,  y1-5,  x2+5,  y2+5,  LFOOD,    7)
    Env.SetRectangleField(x1-10, y1-10, x2+10, y2+10, LDANGER,  200)
#
# Препятствия
#


FoodLightObstField(20, 50, 5)

# place obstacles randomly
for i in range(0, N_OBSTACLES):
    x = random.randint(0, Env.DIM_X)
    y = random.randint(0, Env.DIM_Y)
    Env.SetRectangleField(x-5, y-5, x+5, y+5, gdic.LEVEL_GROUND, 200)
