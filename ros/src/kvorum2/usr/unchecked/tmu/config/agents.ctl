#
# Описание робота tmu
#

NUM_SHARP_FL = 6
NUM_SHARP_FR = 7
NUM_SHARP_CL = 1
NUM_SHARP_CR = 2
NUM_SHARP_LOCATOR = 3 # ??? И как его привязать к A3?
NUM_REFLEX = 27

SHARP_DIST = 180
TSOP_DIST = 140

########################################################################
# Создание робота
########################################################################

def CreateRobot(id, pos, tip):
    print "create", id
    # Аргументы: cid, cpos, cshape, cenv, csize = 0
    A = TAgent(id, pos, tip, Env, csize = 0.75)

    A.traceOn = False

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
    global SHARP_DIST, TSOP_DIST
    global NUM_SHARP_FL, NUM_SHARP_FR, NUM_SHARP_CL, NUM_SHARP_CR, NUM_SHARP_LOCATOR

    # SHARP левый передний
    A.Sensors.append(TSensor(gdic.ADDR_MvCtl, NUM_SHARP_FL, 20,  1, SHARP_DIST, gdic.LEVEL_GROUND, gdic.ST_SHARP))

    # SHARP правый передний
    A.Sensors.append(TSensor(gdic.ADDR_MvCtl, NUM_SHARP_FR, -20, 1, SHARP_DIST, gdic.LEVEL_GROUND, gdic.ST_SHARP))

    # SHARP левый боковой
    A.Sensors.append(TSensor(gdic.ADDR_MvCtl, NUM_SHARP_CL, 45,  1, SHARP_DIST, gdic.LEVEL_GROUND, gdic.ST_SHARP))

    # SHARP правый боковой
    A.Sensors.append(TSensor(gdic.ADDR_MvCtl, NUM_SHARP_CR, -45, 1, SHARP_DIST, gdic.LEVEL_GROUND, gdic.ST_SHARP))

    # Локатор SHARP
    # Значения помещаются в TRobot.Locator
    A.Sensors.append(TSensor(gdic.ADDR_UsrData,  0, 0, 180, SHARP_DIST*2, gdic.LEVEL_GROUND, gdic.ST_SHARP, gdic.RST_VECTOR))

    # Датчики TSOP
    A.Sensors.append(TSensor(gdic.ADDR_RC5Server, 0,   0, 90, TSOP_DIST, gdic.LEVEL_IR,  gdic.ST_DETECTOR, gdic.RST_SCALAR))
    A.Sensors.append(TSensor(gdic.ADDR_RC5Server, 1,  90, 90, TSOP_DIST, gdic.LEVEL_IR,  gdic.ST_DETECTOR, gdic.RST_SCALAR))
    A.Sensors.append(TSensor(gdic.ADDR_RC5Server, 2, -90, 90, TSOP_DIST, gdic.LEVEL_IR,  gdic.ST_DETECTOR, gdic.RST_SCALAR))
    A.Sensors.append(TSensor(gdic.ADDR_RC5Server, 3, 180, 90, TSOP_DIST, gdic.LEVEL_IR,  gdic.ST_DETECTOR, gdic.RST_SCALAR))

    return A

########################################################################
# Роботы
########################################################################

Agents.append(CreateRobot(1, [50, 60, 90], "elefant"))

########################################################################
# Вспомогательные функции для обращения к датчикам
########################################################################

def sGetFL(R):
    global NUM_SHARP_FL
    s = R.agent.MainSensors[NUM_SHARP_FL]
    return s

def sGetFR(R):
    global NUM_SHARP_FR
    s = R.agent.MainSensors[NUM_SHARP_FR]
    return s

def sGetSL(R):
    global NUM_SHARP_CL
    s = R.agent.MainSensors[NUM_SHARP_CL]
    return s

def sGetSR(R):
    global NUM_SHARP_CR
    s = R.agent.MainSensors[NUM_SHARP_CR]
    return s

def sGetReflex(R):
    global NUM_REFLEX
    s = R.agent.MainSensors[NUM_REFLEX]
    return s