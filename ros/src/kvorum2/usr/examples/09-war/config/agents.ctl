#
# flock hunting war_demo
# Описание робота
#

TIP_T1 = "turtle" #"p_yellow" # "turtle"
TIP_T2 = "hare" #"p_red"    # "hare"

NUM_T1 = 10
NUM_T2 = 5

IR_CODE_BASE_T1 =  1
IR_CODE_BASE_T2 = NUM_T1 + 1 + 100

SL_TSOP_FAR_DIST  = 30
SL_TSOP_NEAR_DIST = 10

########################################################################
# Создание робота
########################################################################

def CreateRobot(id, pos, tip, irvalue):
    global TIP_T1, TIP_T2
    global NUM_T1, NUM_T2, IR_CODE_T1, IR_CODE_T2
    global SL_TSOP_FAR_DIST, SL_TSOP_NEAR_DIST

    # Аргументы: cid, cpos, cshape, cenv, csize = 0
    A = TAgent(id, pos, tip, Env, csize = 0.55) # 0.75

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
    # Суперлокатор
    # Работает со слоем gdic.LEVEL_IR
    # Датчик с углом раствора cang и дистанцией DIST
    #
    cang = 180
    DIST = SL_TSOP_FAR_DIST
    A.AddSens(TSensor("superlocator", 0, cang, DIST, gdic.LEVEL_IR, gdic.ST_USONIC, gdic.RST_SUPER_VECTOR))

    # Агент является источником IR-сигнала (gdic.LEVEL_IR)
    A.SetSrc(gdic.LEVEL_IR, irvalue)

    # Крейсерские скорости движения и разворота
    A.SetNormVSpeed(1, 2.5)
    return A

########################################################################
# Роботы
########################################################################

#
# Агенты являются источниками ИК-излучения, код которого не постоянный, а зависит от id агента
#
id = 1
# Агенты 1 группы
x0, y0 = 20, 40
for i in range(0, NUM_T1):
    ircode = IR_CODE_BASE_T1 + i
    Agents.append(CreateRobot(id, [x0+i, y0, random.randint(100, 110)], TIP_T1, ircode))
    print(id, ircode)
    id += 1

# Агенты 2 группы
x0, y0 = 20, 60
for i in range(0, NUM_T2):
    ircode = IR_CODE_BASE_T2 + i
    Agents.append(CreateRobot(id, [x0+i, y0, random.randint(-110, -100)], TIP_T2, ircode))
    print(id, ircode)
    id += 1

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
