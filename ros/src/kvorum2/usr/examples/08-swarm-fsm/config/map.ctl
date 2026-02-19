#
# emo
#
def CField(x, y, dim):
    Env.SetRoundField(x, y, dim, gdic.LEVEL_LIGHT, 200)
    Env.SetRoundField(x, y, dim-5, gdic.LEVEL_COLOR, 200)

def RField(x, y, dim):
    Env.SetRectangleField(x, y, x+dim, y+dim, gdic.LEVEL_LIGHT, 200)
    Env.SetRectangleField(x+10, y+10, x+dim-10, y+dim-10, gdic.LEVEL_COLOR, 200)

#
# Препятствия
#
"""
Env.SetRectangleField(25, 70, 35, 75, gdic.LEVEL_GROUND, 200)
Env.SetRectangleField(40, 20, 50, 25, gdic.LEVEL_GROUND, 200)
Env.SetRectangleField(60, 50, 70, 55, gdic.LEVEL_GROUND, 200)
#Env.SetRectangleField(0, 0, 99, 1, gdic.LEVEL_GROUND, 200)
"""

#
# Вся еда должна быть на свету
#
# Пятна (еда) и Свет (опасность)
#
"""
#CField(70, 5, 15)

CField(10, 15, 10)

#CField(30, 25, 10)

CField(80, 30, 15)

CField(20, 35, 15)

#CField(50, 45, 15)

#CField(10, 55, 15)

CField(50, 65, 15)

CField(80, 75, 15)

CField(20, 80, 15)

#
# IR (просто так)
#
for i in range(0, 10):
    Env.SetFieldVal(15+i, 60, gdic.LEVEL_IR, 200)
    Env.SetFieldVal(15+i, 62, gdic.LEVEL_IR, 200)

"""


RField(0, 49, 50)
RField(49, 0, 50)

