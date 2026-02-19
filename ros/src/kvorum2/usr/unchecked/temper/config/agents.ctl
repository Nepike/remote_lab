#
# gbp
# Описание робота
#

########################################################################
# Создание робота
########################################################################

def CreateRobot(id, pos, tip):
    print "create", id
    # Аргументы: cid, cpos, cshape, cenv, csize = 0
    A = TAgent(id, pos, tip, Env, csize = 0.75)

    #A.traceOn = True

    ##################################################################
    #
    # Сенсоры
    #
    # Аргументы: caddr, cid,  cdir, cang, cR, cLevel, ctip, cvaltype = gdic.ST_SCALAR, cfproc = None, cvalue = 0
    #
    ##################################################################

    #
    # *** Датчики препятствий
    #
    angle = 60
    dir = angle/2
    dist = 40
    # USONIC левый передний
    A.Sensors.append(TSensor(gdic.ADDR_MvCtl, 0,  dir, angle, dist, gdic.LEVEL_GROUND, gdic.ST_USONIC))

    # USONIC правый передний
    A.Sensors.append(TSensor(gdic.ADDR_MvCtl, 1, -dir, angle, dist, gdic.LEVEL_GROUND, gdic.ST_USONIC))

    #
    # *** Датчики освещенности (опасность)
    #
    angle = 60
    dir = angle/2
    dist = 20
    # Глаз левый передний
    A.Sensors.append(TSensor(gdic.ADDR_MvCtl, 5,  dir, angle, dist, gdic.LEVEL_LIGHT, gdic.ST_USONIC))

    # Глаз правый передний
    A.Sensors.append(TSensor(gdic.ADDR_MvCtl, 6, -dir, angle, dist, gdic.LEVEL_LIGHT, gdic.ST_USONIC))

    #
    # *** Датчики цвета (еда)
    #
    # Сенсор пятна (еда). Точечный датчик (радиус=0)
    A.Sensors.append(TSensor(gdic.ADDR_MvCtl, 2, 0, 0, 0, gdic.LEVEL_COLOR, gdic.ST_DETECTOR))

    angle = 60
    dir = angle/2
    dist = 20
    # Глаз левый передний
    A.Sensors.append(TSensor(gdic.ADDR_MvCtl, 3,  dir, angle, dist, gdic.LEVEL_COLOR, gdic.ST_USONIC))

    # Глаз правый передний
    A.Sensors.append(TSensor(gdic.ADDR_MvCtl, 4, -dir, angle, dist, gdic.LEVEL_COLOR, gdic.ST_USONIC))

    return A

########################################################################
# Роботы
########################################################################

random.seed(0)

Agents.append(CreateRobot(1, [30, 60, 180], "elefant"))

