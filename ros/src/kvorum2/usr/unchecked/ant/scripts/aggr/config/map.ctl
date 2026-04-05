
import random

#
# Пятна (еда)
#
"""
for i in range(0, 15):
    x = random.randint(2,195)
    y = random.randint(2,195)
    Env.SetRectangleField(x, y, x+4, y+4, gdic.LEVEL_COLOR, 5)
"""

# Координаты "муравейников"
#X1 = 46
#Y1 = 46
#X2 = 146
#Y2 = 46

size = 4    # размер муравейника
X1 = Env.DIM_X/4-size/2
Y1 = Env.DIM_Y/2-size/2
X2 = 3*Env.DIM_X/4-size/2
Y2 = Env.DIM_Y/2-size/2

Env.SetRectangleField(1, 1, Env.DIM_X/2, Env.DIM_Y, gdic.LEVEL_COLOR, 1)
Env.SetRectangleField(Env.DIM_X/2+1, 1, Env.DIM_X, Env.DIM_Y, gdic.LEVEL_COLOR, 2)

"""
nx = 21
ny = 10
step_x = Env.DIM_X//nx
step_y = Env.DIM_Y//ny
y = 1
for n in range(1, ny):
    y = y + step_y
    x = -5
    for m in range(1, nx):
        x = x+step_x
        x1=x-n//3
        y1=y-m//4
        x2=x-n//3+5
        y2=y-m//4+5
        if x<Env.DIM_X/2: Env.SetRectangleField(x1, y1, x2, y2, gdic.LEVEL_COLOR, 1)
        else: Env.SetRectangleField(x1, y1, x2, y2, gdic.LEVEL_COLOR, 2)
"""
def P(x,y,c,s):
    Env.SetRectangleField(x, y, x+s, y+s, gdic.LEVEL_COLOR, c)

# Очищаем область дома    
#P(X1-5, Y1-5, 0, size+10) 
#P(X2-5, Y2-5, 0, size+10)      

# Рисуем дома
P(X1, Y1, 11, size) # цвет = номер группы + 10
P(X2, Y2, 12, size) # цвет = номер группы + 10      

        
        
        
        
        
        
        