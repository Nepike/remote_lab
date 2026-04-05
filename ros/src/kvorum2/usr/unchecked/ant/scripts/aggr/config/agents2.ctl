#
# Ориентация в пространстве
# Описание робота
#

import math

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
    if (id==100): cs=0.75
    else: cs=0.5
    
    # Аргументы: cid, cpos, cshape, cenv, csize = 0
    A = TAgent(id, pos, tip, Env, csize = cs)
    A.traceOn = False
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
    # A.Sensors.append(TSensor(gdic.ADDR_MvCtl, 2, 45,  1, 10, gdic.LEVEL_GROUND, gdic.ST_USONIC))

    # USONIC правый боковой
    # A.Sensors.append(TSensor(gdic.ADDR_MvCtl, 3, -45, 1, 10, gdic.LEVEL_GROUND, gdic.ST_USONIC))

    # USONIC центральный дальномер
    # A.Sensors.append(TSensor(gdic.ADDR_MvCtl, 4, 0, 1, 20, gdic.LEVEL_GROUND, gdic.ST_USONIC))

    # Локатор
    A.Sensors.append(TSensor(gdic.ADDR_UsrData,  0, 0, 360, 10, gdic.LEVEL_LIGHT, gdic.ST_DETECTOR, gdic.RST_VECTOR))

    #
    # *** Датчики цвета
    #
    # Сенсор пятна (дом). Точечный датчик (радиус=0)
    A.Sensors.append(TSensor(gdic.ADDR_I2CDataServer, 0, 0, 0, 0, gdic.LEVEL_COLOR, gdic.ST_DETECTOR))
    
    # Суперлокатор. Он должен быть один
    #A.Sensors.append(TSensor(gdic.ADDR_SuperLocator,  0, 0, 360, 75, gdic.LEVEL_COLOR, gdic.ST_USONIC, gdic.RST_SUPER_VECTOR))
    
    #
    # *** Прочие датчики
    #
    # Сенсор освещенности. Точечный датчик (радиус=0)
    # A.Sensors.append(TSensor(gdic.ADDR_I2CDataServer, 1, 0, 0, 0, gdic.LEVEL_LIGHT, gdic.ST_DETECTOR))

    # Датчики TSOP
    # TSOP_DIST = 20
    # A.Sensors.append(TSensor(gdic.ADDR_RC5Server, 0,   0, 90, TSOP_DIST, gdic.LEVEL_IR,  gdic.ST_DETECTOR, gdic.RST_SCALAR))
    # A.Sensors.append(TSensor(gdic.ADDR_RC5Server, 1,  90, 90, TSOP_DIST, gdic.LEVEL_IR,  gdic.ST_DETECTOR, gdic.RST_SCALAR))
    # A.Sensors.append(TSensor(gdic.ADDR_RC5Server, 2, -90, 90, TSOP_DIST, gdic.LEVEL_IR,  gdic.ST_DETECTOR, gdic.RST_SCALAR))
    # A.Sensors.append(TSensor(gdic.ADDR_RC5Server, 3, 180, 90, TSOP_DIST, gdic.LEVEL_IR,  gdic.ST_DETECTOR, gdic.RST_SCALAR))
    
    # Агент становится источником сигнала
    A.SetSrc(gdic.LEVEL_LIGHT, id)
    V_DIST = 5
    A.Sensors.append(TSensor(gdic.ADDR_I2CDataServer, 0, 0, 180, V_DIST, gdic.LEVEL_LIGHT, gdic.ST_DETECTOR, gdic.RST_SCALAR))
    
    return A

########################################################################
# Роботы
########################################################################

RAZMER = 100
A = 15
X1 = 50
Y1 = 50
X2 = 50
Y2 = 150
X3 = 150
Y3 = 150
X4 = 150
Y4 = 50
k = RAZMER

for n in range(0, A):
    grad = n*360/A
    fi = math.radians(grad)
    x = X1 + 3*math.cos(fi)
    y = Y1 + 3*math.sin(fi)
    Agents.append(CreateRobot(k, [ x, y, grad ], "turtle"))
    x = X2 + 3*math.cos(fi)
    y = Y2 + 3*math.sin(fi)
    Agents.append(CreateRobot(k+RAZMER, [ x, y, grad ], "elefant"))
    k +=1    
    grad = n*360/(2*A)
    Agents.append(CreateRobot(k+RAZMER, [ x, y, grad ], "elefant"))
    k +=1
    grad = n*360/(3*A)
    Agents.append(CreateRobot(k+RAZMER, [ x, y, grad ], "elefant"))
    k +=1

for n in range(0, A):
    grad = n*360/A
    fi = math.radians(grad)
    x = X1 + 3*math.cos(fi)
    y = Y1 + 3*math.sin(fi)
    Agents.append(CreateRobot(k, [ x, y, grad ], "turtle"))
    k +=1    
    grad = n*360/(2*A)
    Agents.append(CreateRobot(k, [ x, y, grad ], "turtle"))
    x = X2 + 3*math.cos(fi)
    y = Y2 + 3*math.sin(fi)
    Agents.append(CreateRobot(k+RAZMER, [ x, y, grad ], "elefant"))
    k +=1
    

########################################################################
# Вспомогательные функции для обращения к датчикам
########################################################################
