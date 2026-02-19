#
# emo
#
# 
LFOOD = gdic.LEVEL_COLOR
LDANGER = gdic.LEVEL_LIGHT

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
#Env.SetRectangleField(25, 70, 35, 75, gdic.LEVEL_GROUND, 200)
#Env.SetRectangleField(40, 20, 50, 25, gdic.LEVEL_GROUND, 200)
#Env.SetRectangleField(60, 50, 70, 55, gdic.LEVEL_GROUND, 200)


#
# Вся еда должна быть на свету
#
# Пятна (еда) и Свет (опасность)
#

FoodLightObstField(20, 20, 5)

FoodLightObstField(20, 50, 5)

FoodLightObstField(50, 20, 5)

FoodLightObstField(80, 20, 5)

FoodLightObstField(80, 80, 5)

FoodLightObstField(20, 80, 5)

FoodLightObstField(80, 50, 5)

FoodLightObstField(50, 80, 5)

FoodLightObstField(50, 50, 5)
