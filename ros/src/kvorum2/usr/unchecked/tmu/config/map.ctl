
# tmu

# Препятствия
#

#Env.SetRectangleField(25, 70, 35, 75, gdic.LEVEL_GROUND, 200)
#Env.SetRectangleField(15, 10, 25, 13, gdic.LEVEL_GROUND, 200)


#
# IR (просто так)
#
for i in range(0, 10):
    Env.SetFieldVal(15+i, 60, gdic.LEVEL_IR, 200)

#--------------------------------------------------------------------------------------

def G(x,y):
    Env.SetRectangleField(x, y, x+10, y+5, gdic.LEVEL_GROUND, 1)

#G(20, 20)
#G(70, 70)
#G(50, 50)
#G(70, 30)



