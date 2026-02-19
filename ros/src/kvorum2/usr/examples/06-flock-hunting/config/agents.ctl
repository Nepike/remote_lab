#
# flock hunting demo
# Описание робота
#

########################################################################
# Вспомогательная функция для инверсии датчиков
########################################################################
def f_inv(r):
    if(r==0): return 1
    else: return 0

TIP_HUNTER = "turtle"
TIP_VICTIM = "hare"

HUNTER_IR_CODE = 0xA0
VICTIM_IR_CODE = 0x01

########################################################################
# Создание робота
########################################################################

def CreateRobot(id, pos, tip, irvalue):
    global TIP_HUNTER, TIP_VICTIM

    # Аргументы: cid, cpos, cshape, cenv, csize = 0
    A = TAgent(id, pos, tip, Env, csize = 0.75) # 0.65 0.75
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


    # Суперлокатор
    # Работает со слоем gdic.LEVEL_IR
    # Датчик с углом раствора cang и дистанцией DIST
    #
    cang = 120
    DIST = 20
    A.AddSens(TSensor("superlocator", 0, cang, DIST, gdic.LEVEL_IR, gdic.ST_USONIC, gdic.RST_SUPER_VECTOR))

    # Агент становится источником сигнала
    # Это нужно для определения типа агента
    A.SetSrc(gdic.LEVEL_IR, irvalue)

    # Охотник
    if(tip==TIP_HUNTER):
        # Датчики освещенности для охотника. Нужен для обнаружения жертвы в зоне досягаемости и поедания ее
        V_DIST = 5
        A.AddSens(TSensor("light", 0, 180, V_DIST, gdic.LEVEL_LIGHT, gdic.ST_DETECTOR, gdic.RST_SCALAR))

        # Крейсерские скорости движения и разворота
        A.SetNormVSpeed(1, 5)
    else:
        # Жертва является источником. Нужно для того, чтобы хищник мог обнаружить ее в непосредственной близости и
        # съесть ее
        A.SetSrc(gdic.LEVEL_LIGHT, id)

        # Крейсерские скорости движения и разворота
        A.SetNormVSpeed(2, 5)

    return A

########################################################################
# Роботы
########################################################################
NUM_HUNT = 5

NUM_VICTIM = 10

#
# Агенты являются источниками ИК-излучения, код которого не постоянный, а зависит от id агента
#

# Охотники
x0 = 5
for i in range(0, NUM_HUNT):
    Agents.append(CreateRobot(i+1, [x0+i*2, 1, 90], TIP_HUNTER, HUNTER_IR_CODE+i))

# Жертвы
x0 = 5 # 50
for i in range(0, NUM_VICTIM):
    Agents.append(CreateRobot(i+1+NUM_HUNT, [x0+i*2, 80, -90], TIP_VICTIM, VICTIM_IR_CODE+i))


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

