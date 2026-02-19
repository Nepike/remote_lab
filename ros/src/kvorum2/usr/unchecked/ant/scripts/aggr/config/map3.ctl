
# Дом
#Env.SetRectangleField(48, 48, 53, 53, gdic.LEVEL_GROUND, 200)

# Ориентиры

def P(x,y,c):
    #Env.SetFieldVal(x, y, gdic.LEVEL_COLOR, c)
    Env.SetRectangleField(x, y, x+1, y+1, gdic.LEVEL_COLOR, c)

#Дома
P(130, 130, 1)
P(70, 70, 2)

# Препятствия
#Env.SetRectangleField(45, 50, 48, 70, gdic.LEVEL_GROUND, 200)
#Env.SetRectangleField(26, 35, 36, 39, gdic.LEVEL_GROUND, 200)

#Env.SetFieldVal(30, 55, gdic.LEVEL_GROUND, 5)
#Env.SetFieldVal(35, 40, gdic.LEVEL_GROUND, 5)

#Agents.append(CreateRobot(1, [40, 55, 90], "elefant"))
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
