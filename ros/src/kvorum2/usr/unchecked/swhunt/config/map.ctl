#
# Hunting
#
# Препятствия

Env.SetRectangleField(25, 70, 35, 75, gdic.LEVEL_GROUND, 1)
Env.SetRectangleField(25, 20, 35, 30, gdic.LEVEL_GROUND, 1)
Env.SetRectangleField(45, 50, 65, 60, gdic.LEVEL_GROUND, 1)


# Пятна (еда)
for i in range(0, 5):
    Env.SetRoundField(100-(i*15), 5+i*15, 10, gdic.LEVEL_COLOR, 200)

# Свет
Env.SetRoundField(20, 85, 15, gdic.LEVEL_LIGHT, 200)

# IR
for i in range(0, 10):
    Env.SetFieldVal(15+i, 60, gdic.LEVEL_IR, 200)
Env.SetRoundField(50, 50, 5, gdic.LEVEL_IR, 100)
