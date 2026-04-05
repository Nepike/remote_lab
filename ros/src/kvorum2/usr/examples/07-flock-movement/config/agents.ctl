#
# flock moving demo
# Описание робота
#
random.seed(0)

TIP_LEADER = "hare"
TIP_WORKER = "turtle"

########################################################################
# Создание робота
########################################################################

def CreateRobot(id, pos, tip, irvalue):
    global TIP_LEADER, TIP_WORKER

    # Аргументы: cid, cpos, cshape, cenv, csize = 0
    A = TAgent(id, pos, tip, Env, csize = 0.5) # 0.3 0.65 0.75

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


    # Датчики TSOP
    # Датчик с углом раствора cang и дистанцией TSOP_DIST
    cang = 90
    TSOP_DIST = 40

    A.AddSens(TSensor("TSOP_FWD",     0, cang, TSOP_DIST, gdic.LEVEL_IR,  gdic.ST_DETECTOR, gdic.RST_SCALAR))
    A.AddSens(TSensor("TSOP_LEFT",   90, cang, TSOP_DIST, gdic.LEVEL_IR,  gdic.ST_DETECTOR, gdic.RST_SCALAR))
    A.AddSens(TSensor("TSOP_RIGHT", -90, cang, TSOP_DIST, gdic.LEVEL_IR,  gdic.ST_DETECTOR, gdic.RST_SCALAR))
    A.AddSens(TSensor("TSOP_BACK",  180, cang, TSOP_DIST, gdic.LEVEL_IR,  gdic.ST_DETECTOR, gdic.RST_SCALAR))

    # Агент становится источником сигнала
    # Это нужно для определения типа агента
    A.SetSrc(gdic.LEVEL_IR, irvalue)

    # Крейсерские скорости движения и разворота
    if(tip==TIP_WORKER):
        A.SetNormVSpeed(3, 5)
    else:
        A.SetNormVSpeed(1, 5)

    return A

########################################################################
# Роботы
########################################################################
NUM_ROBOTS = 2
WORKER_IR_CODE = 0x01
LEADER_IR_CODE = WORKER_IR_CODE+100

#
# Агенты являются источниками ИК-излучения, код которого не постоянный, а зависит от id агента
#

x0 = Env.DIM_X/2
y0 = Env.DIM_Y/2
id = 1

for i in range(NUM_ROBOTS):
    Agents.append(CreateRobot(id, [x0+i*2, y0, 90], TIP_WORKER, WORKER_IR_CODE+id))
    id+=1
    Agents.append(CreateRobot(id, [x0+i*2, y0+2, 90], TIP_WORKER, WORKER_IR_CODE+id))
    id+=1
    Agents.append(CreateRobot(id, [x0+i*2, y0+4, 90], TIP_WORKER, WORKER_IR_CODE+id))
    id+=1

# Лидер
Agents.append(CreateRobot(255, [x0, y0+20, 90], TIP_LEADER, LEADER_IR_CODE))

########################################################################
#
# Вспомогательные функции для обращения к датчикам
#
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
