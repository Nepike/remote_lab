#
# demo
# Описание робота
#

########################################################################
# Создание робота
########################################################################

#
#
#
def CreateRobot(id, pos, tip, show_id):
    print('create {}'.format(id))
 
    # Аргументы: cid, cpos, cshape, cenv, show_id=False, csize = 0
    A = TAgent(id, pos, tip, Env, show_id, csize = 0.75)

    # Режим трассировки движения
    A.traceOn = True
    A.DrawSensors = True

    ##################################################################
    #
    # Сенсоры
    #
    # Аргументы: cname,  cdir, cang, cR, cLevel, ctip, cvaltype = gdic.ST_SCALAR, cfproc = None, cval = 0
    #    cname -  Имя
    #    cdir  -  Направление относительно робота (x, y, dir)
    #    cang  -  Угол раствора
    #    cR    -  Радиус действия. Если R=0, то это - точечный датчик
    #    cLevel - Рабочий слой сенсора
    #    ctip   - Тип датчика: gdic.ST_SHARP, gdic.ST_USONIC, gdic.ST_DETECTOR, gdic.ST_CONST
    #    cvaltype - Тип результата
    #    cfproc   - Функция постобработки сигнала (или None)
    #    cval     - Значение датчика для типа ST_CONST
    #
    ##################################################################

    #
    # *** Датчики препятствий
    #
    # USONIC левый передний
    USONIC_DIST = 10
    A.AddSens(TSensor("RF_LEFT", 20,  1, USONIC_DIST, gdic.LEVEL_GROUND, gdic.ST_USONIC))

    # USONIC правый передний
    A.AddSens(TSensor("RF_RIGHT", -20, 1, USONIC_DIST, gdic.LEVEL_GROUND, gdic.ST_USONIC))

    TSOP_DIST = 20

    #
    # Датчик с углом раствора cang и дистанцией TSOP_DIST
    #
    cang = 10
    A.AddSens(TSensor("IR_SENSOR", 0, cang, TSOP_DIST, gdic.LEVEL_IR, gdic.ST_DETECTOR))

    # Датчики TSOP
    cang = 90
    A.AddSens(TSensor("TSOP_FWD",     0, cang, TSOP_DIST, gdic.LEVEL_IR,  gdic.ST_DETECTOR, gdic.RST_SCALAR))
    A.AddSens(TSensor("TSOP_LEFT",   90, cang, TSOP_DIST, gdic.LEVEL_IR,  gdic.ST_DETECTOR, gdic.RST_SCALAR))
    A.AddSens(TSensor("TSOP_RIGHT", -90, cang, TSOP_DIST, gdic.LEVEL_IR,  gdic.ST_DETECTOR, gdic.RST_SCALAR))
    A.AddSens(TSensor("TSOP_BACK",  180, cang, TSOP_DIST, gdic.LEVEL_IR,  gdic.ST_DETECTOR, gdic.RST_SCALAR))

    #
    # Точечный датчик (радиус=0)
    #
    A.AddSens(TSensor("IR_DETECTOR", 0, 0, 0, gdic.LEVEL_IR, gdic.ST_DETECTOR, gdic.RST_SCALAR))

    #
    # Суперлокатор
    # Работает со слоем gdic.LEVEL_IR
    cang=360
    DIST=100
    A.AddSens(TSensor("superlocator", 0, cang, DIST, gdic.LEVEL_IR, gdic.ST_USONIC, gdic.RST_SUPER_VECTOR))

    #
    # Агент является источником сигнала id на уровне gdic.LEVEL_IR
    #
    A.SetSrc(gdic.LEVEL_IR, id)

    #
    # Крейсерские скорости движения и разворота
    #
    A.SetNormVSpeed(0.5, 2.5)

    return A

########################################################################
# Роботы
########################################################################

Agents.append(CreateRobot(1, [50, 50, 90], "turtle", show_id=True))

Agents.append(CreateRobot(2, [50, 70, 180], "elefant", show_id=True))
Agents.append(CreateRobot(3, [30, 50, 180], "elefant", show_id=True))
Agents.append(CreateRobot(4, [70, 50, 180], "elefant", show_id=True))
Agents.append(CreateRobot(5, [50, 30, 180], "elefant", show_id=True))
