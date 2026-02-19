
# demo

# Препятствия
#
for i in range(0, 10):
    Env.SetFieldVal(30+i, 40+i, gdic.LEVEL_GROUND, 1)
    Env.SetFieldVal(31+i, 40+i, gdic.LEVEL_GROUND, 1)

Env.SetRectangleField(25, 70, 35, 75, gdic.LEVEL_GROUND, 200)

Env.SetRectangleField(15, 10, 25, 13, gdic.LEVEL_GROUND, 200)

#
# Пятна
#
Env.SetRoundField(10, 15, 3, gdic.LEVEL_COLOR, 1)
Env.SetRoundField(20, 15, 3, gdic.LEVEL_COLOR, 2)
Env.SetRoundField(30, 15, 3, gdic.LEVEL_COLOR, 3)


Env.SetRoundField(70,  5, 3, gdic.LEVEL_COLOR, 4)
Env.SetRoundField(30, 25, 3, gdic.LEVEL_COLOR, 5)
Env.SetRoundField(80, 30, 3, gdic.LEVEL_COLOR, 6)
Env.SetRoundField(20, 35, 3, gdic.LEVEL_COLOR, 7)
Env.SetRoundField(50, 45, 3, gdic.LEVEL_COLOR, 8)
Env.SetRoundField(10, 55, 3, gdic.LEVEL_COLOR, 9)
Env.SetRoundField(50, 65, 3, gdic.LEVEL_COLOR, 10)
Env.SetRoundField(80, 75, 3, gdic.LEVEL_COLOR, 11)
Env.SetRoundField(20, 85, 3, gdic.LEVEL_COLOR, 12)


#
# Свет
#
Env.SetRoundField(50, 65, 10, gdic.LEVEL_LIGHT, 200)
Env.SetRoundField(80, 75, 10, gdic.LEVEL_LIGHT, 200)
Env.SetRoundField(20, 85, 10, gdic.LEVEL_LIGHT, 200)

Env.SetRoundField(22, 5, 5, gdic.LEVEL_LIGHT, 200)

#
# IR (просто так)
#
for i in range(0, 10):
    Env.SetFieldVal(15+i, 60, gdic.LEVEL_IR, 200)

#--------------------------------------------------------------------------------------

def P(x,y,c):
    Env.SetRectangleField(x, y, x+10, y+20, gdic.LEVEL_COLOR, c)

def G(x,y):
    Env.SetRectangleField(x, y, x+10, y+5, gdic.LEVEL_GROUND, 1)
xr = 50
yr = 30

Env.SetLineField(0, 100, -5, 50, gdic.LEVEL_GROUND, 1)
#Env.SetLineField(50, 30, 180, 20, gdic.LEVEL_GROUND, 1)

#Env.SetRectangleField(xr-10, yr, xr-11, yr+3, gdic.LEVEL_COLOR, 3)
#Env.SetRectangleField(xr-10, yr, xr-21, yr+3, gdic.LEVEL_COLOR, 3)

P(30, 20, 3)
P(40, 40, 4)
P(51, 40, 5)
P(60, 20, 6)

P(20, 40, 1)
P(30, 40, 2)
P(40, 40, 3)

P(20, 30, 4)
P(40, 30, 5)

P(20, 20, 6)
P(30, 20, 7)
P(40, 20, 8)

G(20, 20)
G(70, 70)
G(50, 50)
G(70, 30)


Env.SetRectangleField(50, 65, 51, 66, gdic.LEVEL_COLOR, 6)

Env.SetLineField(50, 50, 10, 20, gdic.LEVEL_COLOR, 1)
Env.SetLineField(50, 50, 170, 20, gdic.LEVEL_GROUND, 1)

Env.SetLineField(50, 50, 80, 20, gdic.LEVEL_COLOR, 2)
Env.SetLineField(50, 50, 100, 20, gdic.LEVEL_COLOR, 3)

Env.Text(10, 10, "text 10x10")
Env.Text(90, 50, "text 90x50")
Env.Text(50, 90, "text 50x90")
