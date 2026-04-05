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

    # Аргументы: cid, cpos, cshape, cenv, show_id=False, csize = 0
    A = TAgent(id, pos, tip, Env, show_id, csize = 0.5)

    # Режим трассировки движения
    A.traceOn = False
    A.DrawSensors = False

    ##################################################################
    #
    # Сенсоры
    #
    # Аргументы: cname,  cdir, cang, cR, cLevel, ctip, cvaltype = gdic.ST_SCALAR, cfproc = None, cval = 0
    #    cname  - Имя датчика
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

    # *** Слой LEVEL_GROUND
    # *** Датчики препятствий
    # Датчик с углом раствора cang и дистанцией dist
    #
    cang = 10
    dist = 20

    # USONIC центральный передний
    A.AddSens(TSensor("RF_FWD_CENTER", 0,  cang, dist, gdic.LEVEL_GROUND, gdic.ST_USONIC))

    # USONIC левый передний
    A.AddSens(TSensor("RF_FWD_LEFT", 20,  cang, dist, gdic.LEVEL_GROUND, gdic.ST_USONIC))

    # USONIC правый передний
    A.AddSens(TSensor("RF_FWD_RIGHT", -20,  cang, dist, gdic.LEVEL_GROUND, gdic.ST_USONIC))

    # USONIC боковой левый передний
    A.AddSens(TSensor("RF_SIDE_FWD_LEFT", 45,  cang, dist, gdic.LEVEL_GROUND, gdic.ST_USONIC))

    # USONIC боковой правый передний
    A.AddSens(TSensor("RF_SIDE_FWD_RIGHT", -45,  cang, dist, gdic.LEVEL_GROUND, gdic.ST_USONIC))

    # USONIC боковой левый задний
    A.AddSens(TSensor("RF_SIDE_BACK_LEFT", 135,  cang, dist, gdic.LEVEL_GROUND, gdic.ST_USONIC))

    # USONIC боковой правый задний
    A.AddSens(TSensor("RF_SIDE_BACK_RIGHT", -135,  cang, dist, gdic.LEVEL_GROUND, gdic.ST_USONIC))


    # Датчики TSOP
    # Датчик с углом раствора cang и дистанцией TSOP_DIST
    TSOP_DIST = 20
    cang = 90
    A.AddSens(TSensor("TSOP_FWD",     0, cang, TSOP_DIST, gdic.LEVEL_IR,  gdic.ST_DETECTOR, gdic.RST_SCALAR))
    A.AddSens(TSensor("TSOP_LEFT",   90, cang, TSOP_DIST, gdic.LEVEL_IR,  gdic.ST_DETECTOR, gdic.RST_SCALAR))
    A.AddSens(TSensor("TSOP_RIGHT", -90, cang, TSOP_DIST, gdic.LEVEL_IR,  gdic.ST_DETECTOR, gdic.RST_SCALAR))
    A.AddSens(TSensor("TSOP_BACK",  180, cang, TSOP_DIST, gdic.LEVEL_IR,  gdic.ST_DETECTOR, gdic.RST_SCALAR))

    #
    # Точечный датчик (радиус=0)
    #
    A.AddSens(TSensor("MINE_DETECTOR", 0, 0, 0, gdic.LEVEL_COLOR, gdic.ST_DETECTOR))

    #
    # Суперлокатор
    # Работает со слоем gdic.LEVEL_IR
    # Датчик с углом раствора cang и дистанцией DIST
    #
    cang = 120
    DIST = 20
    A.AddSens(TSensor("superlocator", 0, cang, DIST, gdic.LEVEL_GROUND, gdic.ST_USONIC, gdic.RST_SUPER_VECTOR))

    #
    # Агент является источником сигнала id на уровне gdic.LEVEL_IR
    #
    A.SetSrc(gdic.LEVEL_IR, id)

    #
    # Крейсерские скорости движения и разворота
    #
    A.SetNormVSpeed(1, 2.5)
    return A

########################################################################
# Роботы
########################################################################
#
# В результате работы утилиты map2ctl в файле map.ctl были определены такие переменные:
#   _RobotNum_ -- количество роботов
#   _RobotX_   -- список координат X
#   _RobotY_   -- список координат Y
#   _RobotA_   -- список углов ориентации
#

print("===================================")
print(_RobotNum_, _RobotX_, _RobotY_, _RobotA_)

for n in range(_RobotNum_):
    print(f"Create robot {n} at pos=({_RobotX_[n]}, {_RobotY_[n]}), ang={_RobotA_[n]}")

    # Для разнообразия:
    # figure = "turtle", "elefant", "hare", "square", "yarp"

    if _RobotA_[n]==0: figure = "hare"
    elif _RobotA_[n]==180: figure = "elefant"
    else: figure = "turtle"

    Agents.append(CreateRobot(n+1, [_RobotX_[n], _RobotY_[n], _RobotA_[n]], figure, show_id=True))

print("===================================")
