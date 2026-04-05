
import random

# Ориентиры

def P(x,y,c):
    Env.SetRectangleField(x, y, x+1, y+1, gdic.LEVEL_COLOR, c)

#Дома
P(67, 133, 5)
P(133, 67, 6)

# Препятствия
#Env.SetRectangleField(45, 50, 48, 70, gdic.LEVEL_GROUND, 200)
#Env.SetRectangleField(26, 35, 36, 39, gdic.LEVEL_GROUND, 200)

#
# Пятна (еда)
#
"""
for i in range(0, 15):
    x = random.randint(2,195)
    y = random.randint(2,195)
    Env.SetRectangleField(x, y, x+4, y+4, gdic.LEVEL_COLOR, 5)
"""

nx = 10
ny = 10
step_x = Env.DIM_X//nx
step_y = Env.DIM_X//ny
y = +5
for n in range(1, ny):
    y = y + step_y
    x = -5
    for m in range(1, nx):
        x = x+step_x
        if x<y: Env.SetRectangleField(x-n//6, y-m//7, x-n//6+5, y-m//7+5, gdic.LEVEL_COLOR, 1)

y = -5
for n in range(1, ny):
    y = y + step_y
    x = +5
    for m in range(1, nx):
        x = x+step_x
        if x>y: Env.SetRectangleField(x-n//6, y-m//7, x-n//6+5, y-m//7+5, gdic.LEVEL_COLOR, 2)
