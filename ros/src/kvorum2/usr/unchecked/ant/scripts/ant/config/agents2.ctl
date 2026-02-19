#
# Темперамент и эмоции
# Описание робота
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
    A.DrawSensors = False

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
    # USONIC левый передний
    A.Sensors.append(TSensor(gdic.ADDR_MvCtl, 0, 20,  1, 10, gdic.LEVEL_GROUND, gdic.ST_USONIC))

    # USONIC правый передний
    A.Sensors.append(TSensor(gdic.ADDR_MvCtl, 1, -20, 1, 10, gdic.LEVEL_GROUND, gdic.ST_USONIC))

    # USONIC левый боковой
    A.Sensors.append(TSensor(gdic.ADDR_MvCtl, 2, 45,  1, 10, gdic.LEVEL_GROUND, gdic.ST_USONIC))

    # USONIC правый боковой
    A.Sensors.append(TSensor(gdic.ADDR_MvCtl, 3, -45, 1, 10, gdic.LEVEL_GROUND, gdic.ST_USONIC))

    # USONIC центральный дальномер
    A.Sensors.append(TSensor(gdic.ADDR_MvCtl, 4, 0, 1, 20, gdic.LEVEL_GROUND, gdic.ST_USONIC))

    # Локатор
    A.Sensors.append(TSensor(gdic.ADDR_UsrData,  0, 0, 90, 20, gdic.LEVEL_GROUND, gdic.ST_USONIC, gdic.RST_VECTOR))

    #
    # *** Датчики цвета
    #
    # Сенсор пятна (еда). Точечный датчик (радиус=0)
    A.Sensors.append(TSensor(gdic.ADDR_I2CDataServer, 0, 0, 0, 0, gdic.LEVEL_COLOR, gdic.ST_DETECTOR))
    
    # Суперлокатор. Он должен быть один
    A.Sensors.append(TSensor(gdic.ADDR_SuperLocator,  0, 0, 180, 20, gdic.LEVEL_COLOR, gdic.ST_USONIC, gdic.RST_SUPER_VECTOR))
    
    #
    # *** Прочие датчики
    #
    # Сенсор освещенности. Точечный датчик (радиус=0)
    A.Sensors.append(TSensor(gdic.ADDR_I2CDataServer, 1, 0, 0, 0, gdic.LEVEL_LIGHT, gdic.ST_DETECTOR))

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

#Agents.append(CreateRobot(1, [20, 5, 90], "elefant"))
Agents.append(CreateRobot(1, [30, 30, 90], "elefant"))

########################################################################
# Вспомогательные функции для обращения к датчикам
########################################################################

def sGetFL(A):
    s = A.agent.MainSensors[0]
    if(s==0): s = 100
    return s
  
def sGetFR(A):
    s = A.agent.MainSensors[1]
    if(s==0): s = 100
    return s

def sGetSL(A):
    s = A.agent.MainSensors[2]
    if(s==0): s = 100
    return s

def sGetSR(A):
    s = A.agent.MainSensors[3]
    if(s==0): s = 100
    return s
  
def sGetC(A):
    s = A.agent.MainSensors[4]
    if(s==0): s = 100
    return s
  
def sGetColor(A):
    s = A.Dataserver[0]
    if(s==0): s = 100
    return s
  
def sGetLight(A):
    s = A.Dataserver[1]
    if(s==0): s = 100
    return s
