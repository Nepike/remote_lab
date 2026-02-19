#
# Hunting
#
########################################################################
# Вспомогательная функция для инверсии датчиков
########################################################################
def f_inv(r):
    if(r==0): return 1
    else: return 0

########################################################################
# Создание робота
########################################################################

def CreateRobot(id, pos, tip):

    # Аргументы: cid, cpos, cshape, cenv, csize = 0
    A = TAgent(id, pos, tip, Env, csize = 0.75)

    ####################################
    # Сенсоры
    # Аргументы: caddr, cid,  cdir, cang, cR, cLevel, ctip, cvaltype = gdic.RST_SCALAR, cfproc = None
    ####################################

    # Sharp левый передний
    A.Sensors.append(TSensor(gdic.ADDR_MvCtl, gdic.REC_SHARP_FL, 20,  2, 10, gdic.LEVEL_GROUND, gdic.ST_SHARP))

    # Sharp правый передний
    A.Sensors.append(TSensor(gdic.ADDR_MvCtl, gdic.REC_SHARP_FR, -20, 2, 10, gdic.LEVEL_GROUND, gdic.ST_SHARP))

    # Sharp левый боковой
    A.Sensors.append(TSensor(gdic.ADDR_MvCtl, gdic.REC_SHARP_SL, 45,  2, 10, gdic.LEVEL_GROUND, gdic.ST_SHARP))

    # Sharp правый боковой
    A.Sensors.append(TSensor(gdic.ADDR_MvCtl, gdic.REC_SHARP_SR, -45, 2, 10, gdic.LEVEL_GROUND, gdic.ST_SHARP))

    # USONIC центральный дальномер
    A.Sensors.append(TSensor(gdic.ADDR_MvCtl, gdic.REC_SHARP_C, 0, 2, 20, gdic.LEVEL_GROUND, gdic.ST_USONIC))


    # Сенсор пятна. Точечный датчик (радиус=0)
    A.Sensors.append(TSensor(gdic.ADDR_I2CDataServer, gdic.REC_SPOT, 0, 0, 0, gdic.LEVEL_COLOR, gdic.ST_DETECTOR))

    # Сенсор освещенности. Точечный датчик (радиус=0)
    A.Sensors.append(TSensor(gdic.ADDR_I2CDataServer, gdic.REC_LIGHT, 0, 0, 0, gdic.LEVEL_LIGHT, gdic.ST_DETECTOR))

    # Локатор
    A.Sensors.append(TSensor(gdic.ADDR_UsrData,  0, 0, 90, 20, gdic.LEVEL_GROUND, gdic.ST_SHARP, gdic.RST_VECTOR))

    # Датчики TSOP
    TSOP_DIST = 20

    A.Sensors.append(TSensor(gdic.ADDR_RC5Server, 0,   0, 90, TSOP_DIST, gdic.LEVEL_IR,  gdic.ST_DETECTOR, gdic.RST_SCALAR))
    A.Sensors.append(TSensor(gdic.ADDR_RC5Server, 1,  90, 90, TSOP_DIST, gdic.LEVEL_IR,  gdic.ST_DETECTOR, gdic.RST_SCALAR))
    A.Sensors.append(TSensor(gdic.ADDR_RC5Server, 2, -90, 90, TSOP_DIST, gdic.LEVEL_IR,  gdic.ST_DETECTOR, gdic.RST_SCALAR))
    A.Sensors.append(TSensor(gdic.ADDR_RC5Server, 3, 180, 90, TSOP_DIST, gdic.LEVEL_IR,  gdic.ST_DETECTOR, gdic.RST_SCALAR))

    return A

########################################################################
# Роботы
########################################################################

NumAgn = 50
Agents.append(CreateRobot(1, [20, 40, 90], "elefant"))
for i in range(1, NumAgn):
    Agents.append(CreateRobot(i+1, [25+i,  5, 90], "turtle"))


"""
Agents.append(CreateRobot(2, [10,  5, 90], "hare"))
Agents.append(CreateRobot(3, [15,  5, 90], "turtle"))
Afind(4).SetSrc(gdic.LEVEL_IR, 0x1A)
Afind(4).SetSrc(gdic.LEVEL_COLOR, 200)
"""
