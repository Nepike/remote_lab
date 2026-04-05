#
# demo
# Описание робота
#

########################################################################
# Создание робота
########################################################################

def CreateRobot(id, pos, tip, show_id):
    print(f"create {id}")
    # Аргументы: cid, cpos, cshape, cenv, show_id=False, csize = 0
    A = TAgent(id, pos, tip, Env, show_id, csize = 0.5)

    # Режим трассировки движения
    A.traceOn = False
    A.DrawSensors = False
    ##################################################################
    #
    # Сенсоры
    #
    # Аргументы: caddr, cid,  cdir, cang, cR, cLevel, ctip, cvaltype = gdic.ST_SCALAR, cfproc = None, cvalue = 0
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


    # Локатор
    A.AddSens(TSensor("locator", 0, 180, 20, gdic.LEVEL_COLOR, gdic.ST_USONIC, gdic.RST_VECTOR))

    #
    # *** Датчики цвета
    #
    # Сенсор пятна (пища). Точечный датчик (радиус=0)
    A.AddSens(TSensor("color", 0, 0, 0, gdic.LEVEL_COLOR, gdic.ST_DETECTOR))

    #
    # Суперлокатор
    # Работает со слоем gdic.LEVEL_COLOR
    # Датчик с углом раствора cang и дистанцией DIST
    #
    cang = 120
    DIST = 20
    A.AddSens(TSensor("superlocator", 0, cang, DIST, gdic.LEVEL_COLOR, gdic.ST_USONIC, gdic.RST_SUPER_VECTOR))

    # Датчики TSOP
    # Датчик с углом раствора cang и дистанцией TSOP_DIST
    cang = 90
    TSOP_DIST = 20
    A.AddSens(TSensor("TSOP_FWD",     0, cang, TSOP_DIST, gdic.LEVEL_IR,  gdic.ST_DETECTOR, gdic.RST_SCALAR))
    A.AddSens(TSensor("TSOP_LEFT",   90, cang, TSOP_DIST, gdic.LEVEL_IR,  gdic.ST_DETECTOR, gdic.RST_SCALAR))
    A.AddSens(TSensor("TSOP_RIGHT", -90, cang, TSOP_DIST, gdic.LEVEL_IR,  gdic.ST_DETECTOR, gdic.RST_SCALAR))
    A.AddSens(TSensor("TSOP_BACK",  180, cang, TSOP_DIST, gdic.LEVEL_IR,  gdic.ST_DETECTOR, gdic.RST_SCALAR))

    #
    # *** Прочие датчики
    #
    # Сенсор освещенности. Точечный датчик (радиус=0)
    A.AddSens(TSensor("light", 0, 0, 0, gdic.LEVEL_LIGHT, gdic.ST_DETECTOR))

    return A

########################################################################
# Роботы
########################################################################

Agents.append(CreateRobot(1, [50, 60, 180], tip="elefant", show_id=True))

########################################################################
# Вспомогательные функции для обращения к датчикам
########################################################################

def sGetFL(A):
    s = A.GetSensor("RF_FWD_LEFT")
    if(s==0): s = 100
    return s

def sGetFR(A):
    s = A.GetSensor("RF_FWD_RIGHT")
    if(s==0): s = 100
    return s

def sGetSL(A):
    s = A.GetSensor("RF_SIDE_FWD_LEFT")
    if(s==0): s = 100
    return s

def sGetSR(A):
    s = A.GetSensor("RF_SIDE_FWD_RIGHT")
    if(s==0): s = 100
    return s

def sGetC(A):
    s = A.GetSensor("RF_FWD_CENTER")
    if(s==0): s = 100
    return s

def sGetColor(A):
    s = A.GetSensor("color")
    if(s==0): s = 100
    return s

def sGetLight(A):
    s = A.GetSensor("light")
    if(s==0): s = 100
    return s
