# demo
# Описание среды
#

# Препятствия
#
for i in range(0, 10):
    Env.SetFieldVal(30+i, 40+i, gdic.LEVEL_GROUND, 1)
    Env.SetFieldVal(31+i, 40+i, gdic.LEVEL_GROUND, 1)

Env.SetRectangleField(25, 70, 35, 75, gdic.LEVEL_GROUND, 200)

Env.SetRectangleField(15, 10, 25, 13, gdic.LEVEL_GROUND, 200)

Env.Text(50, 20, "Some text")

