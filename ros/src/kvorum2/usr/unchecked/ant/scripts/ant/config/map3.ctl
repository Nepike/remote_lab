
# Дом
#Env.SetRectangleField(48, 48, 53, 53, gdic.LEVEL_GROUND, 200)

# Ориентиры

def P(x,y,c):
    #Env.SetFieldVal(x, y, gdic.LEVEL_COLOR, c)
    Env.SetRectangleField(x, y, x+1, y+1, gdic.LEVEL_COLOR, c)

"""
k = 1
for n in range(0, 9):
    for m in range(0, 6):
        P((n+1)*10, (m+1)*15, k)
        k += 1
"""
P(85, 5, 17)
P(20, 75, 2)
P(30, 85, 3)

P(40, 70, 4)
P(50, 85, 5)

P(60, 65, 6)
P(70, 85, 7)
P(80, 65, 8)

P(75, 95, 9)
P(15, 55, 10)

P(75, 75, 11)
P(60, 75, 12)
P(20, 95, 13)

P(20, 45, 14)
P(30, 50, 15)
P(65, 40, 16)

P(15, 30, 20)
P(20, 20, 25)
P(30, 35, 23)
P(40, 25, 28)
P(50, 35, 22)
P(60, 55, 26)

P(85, 50, 21)
P(80, 30, 24)
P(55, 22, 29)

P(10, 10, 30)
P(25, 10, 31)
P(40, 10, 32)
P(55, 10, 33)
P(70, 10, 34)
P(80, 90, 35)

P(5, 30, 36)
P(5, 45, 37)
P(5, 60, 38)
P(5, 75, 39)
P(5, 90, 40)
P(5, 10, 41)

P(30, 63, 42)
P(40, 65, 43)
P(20, 66, 44)
#Объект
#P(15, 95, 1)
P(90, 80, 1)
P(10, 15, 1)
P(60, 65, 1)

#Agents.append(CreateRobot(1, [50, 55, 90], "elefant"))
#Agents.append(CreateRobot(2, [48, 55, 90], "turtle"))

#------------------------------------------------------------
#
# Препятствия
#
#for i in range(0, 40):
#    Env.SetFieldVal(30+i, 40+i, gdic.LEVEL_GROUND, 1)
#    Env.SetFieldVal(31+i, 40+i, gdic.LEVEL_GROUND, 1)

#Env.SetRectangleField(45, 20, 50, 25, gdic.LEVEL_GROUND, 1)

#Env.SetRectangleField(15, 10, 25, 13, gdic.LEVEL_GROUND, 200)

#
# Пятна
#
#Env.SetRoundField(10, 15, 3, gdic.LEVEL_COLOR, 1)
#Env.SetRoundField(20, 15, 3, gdic.LEVEL_COLOR, 2)
#Env.SetRoundField(30, 15, 3, gdic.LEVEL_COLOR, 3)

#Env.SetRoundField(70,  5, 3, gdic.LEVEL_COLOR, 4)
#Env.SetRoundField(30, 25, 3, gdic.LEVEL_COLOR, 5)
#Env.SetRoundField(80, 30, 3, gdic.LEVEL_COLOR, 6)
#Env.SetRoundField(20, 35, 3, gdic.LEVEL_COLOR, 7)
#Env.SetRoundField(50, 45, 3, gdic.LEVEL_COLOR, 8)
#Env.SetRoundField(10, 55, 3, gdic.LEVEL_COLOR, 9)
#Env.SetRoundField(50, 65, 3, gdic.LEVEL_COLOR, 10)
#Env.SetRoundField(80, 75, 3, gdic.LEVEL_COLOR, 11)
#Env.SetRoundField(20, 85, 3, gdic.LEVEL_COLOR, 12)


#
# Свет
#
#Env.SetRoundField(50, 65, 10, gdic.LEVEL_LIGHT, 200)
#Env.SetRoundField(80, 75, 10, gdic.LEVEL_LIGHT, 200)
#Env.SetRoundField(20, 85, 10, gdic.LEVEL_LIGHT, 200)

#Env.SetRoundField(22, 5, 5, gdic.LEVEL_LIGHT, 200)

#
# IR (просто так)
#
#for i in range(0, 10):
#    Env.SetFieldVal(15+i, 60, gdic.LEVEL_IR, 200)

#--------------------------------------------------------------------------------------
