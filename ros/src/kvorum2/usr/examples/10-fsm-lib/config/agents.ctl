#
# demo
# Описание робота
#

########################################################################
# Создание робота
########################################################################

def CreateRobot(id, pos, tip):
    print('create '.format(id))
    # Аргументы: cid, cpos, cshape, cenv, csize = 0
    A = TAgent(id, pos, tip, Env, csize = 0.75)

    A.traceOn = True

    ##################################################################
    #
    # Сенсоры
    #
    # Аргументы: cname, cid,  cdir, cang, cR, cLevel, ctip, cvaltype = gdic.ST_SCALAR, cfproc = None, cvalue = 0
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


    #
    # *** Датчики цвета поверхности (еда) и света
    #
    # Сенсор пятна (еда). Точечный датчик (радиус=0)
    A.AddSens(TSensor("food_detector", 0, 0, 0, gdic.LEVEL_COLOR, gdic.ST_DETECTOR))

    #
    # Суперлокатор
    # Работает со слоем gdic.LEVEL_IR
    # Датчик с углом раствора cang и дистанцией DIST
    #
    cang = 180
    DIST = 20
    A.AddSens(TSensor("superlocator", 0, cang, DIST, gdic.LEVEL_COLOR, gdic.ST_USONIC, gdic.RST_SUPER_VECTOR))

    #
    # *** Прочие датчики
    #
    # Сенсор света. Точечный датчик (радиус=0)
    A.AddSens(TSensor("light", 0, 0, 0, gdic.LEVEL_LIGHT, gdic.ST_DETECTOR))

    # Датчики TSOP
    # Датчик с углом раствора cang и дистанцией TSOP_DIST
    TSOP_DIST = 40
    cang = 90

    A.AddSens(TSensor("TSOP_FWD",     0, cang, TSOP_DIST, gdic.LEVEL_IR,  gdic.ST_DETECTOR, gdic.RST_SCALAR))
    A.AddSens(TSensor("TSOP_LEFT",   90, cang, TSOP_DIST, gdic.LEVEL_IR,  gdic.ST_DETECTOR, gdic.RST_SCALAR))
    A.AddSens(TSensor("TSOP_RIGHT", -90, cang, TSOP_DIST, gdic.LEVEL_IR,  gdic.ST_DETECTOR, gdic.RST_SCALAR))
    A.AddSens(TSensor("TSOP_BACK",  180, cang, TSOP_DIST, gdic.LEVEL_IR,  gdic.ST_DETECTOR, gdic.RST_SCALAR))

    return A

########################################################################
# Роботы
########################################################################

Agents.append(CreateRobot(1, [50, 60, 180], "elefant"))

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

def GetLightSensor(A): return A.GetSensor("light")


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
