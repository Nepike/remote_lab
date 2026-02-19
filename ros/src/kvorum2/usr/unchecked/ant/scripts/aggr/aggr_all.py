#!/usr/bin/env python
# coding: utf-8
"""
  aggr.py

  22.02.17
  Version 1.01
  LP 28.02.2017

"""

import os, sys, time, math

sys.path.append("../../pylib")
sys.path.append("src/kvorum/pylib")

import roslib, rospy, copy, random

import gdic, rcproto, tmurobot as tmu

# Подгружаем модуль именно таким образом, чтобы не было проблем с видимостью
exec (open("src/kvorum/pylib/fsm.py").read())

from env import TEnv
from agent import TAgent, TSensor

Title = "Aggr 1.03"

###################################################################

MAXSTEPS = 100  # количество шагов моделирования
RAZMER = 100    # Максимальное количество роботов в группе

# Задание начального значения random
random.seed(61)  

FoodColor1  = 1  # цвет еды!!!!!!!!!!!!!!!!!!!!!!!!!!
FoodColor2  = 2  # цвет еды!!!!!!!!!!!!!!!!!!!!!!!!!!
GlobalTimer = 0
Env = None
Agents = []
CurrAgent = None

# Глобалы
DEBUG = "DEBUG"
WORK  = "WORK"    # режим работы: печатает только данные о путях

JobREGIM = WORK  # режим отладки: печатает все

# Расчет агрессивности
ONLYDIST = "ONLYDIST"
TIMEDIST = "TIMEDIST"
Arule = ONLYDIST

# Текущий автомат, список автоматов
FSM = None
FSMList = []

# искомый объект
ObjX = [ 0, 0, 0 ] # текущий объект (направление, расстояние до объекта, цвет)
ObjFound = False   # признак того, что робот нашел объект
ObjColor = 1

MyHome = [ 0, 0, 0 ] # дом как объект: направление, расстояние, цвет 

# Состояния агента
S_WALKING  = 0
S_FORAGING = 1

# Расстояние: вплотную, близко, не далеко/не близко, далеко, очень далеко (> DIST_4)
DIST_1 = 2
DIST_2 = 5
DIST_3 = 10
DIST_4 = 15

NEAR     = "NEAR"
CLOSE    = "CLOSE"
NOTCLOSE = "NOTCLOSE"
FAR      = "FAR"
VERYFAR  = "VERYFAR"

D_NEAR     = 1
D_CLOSE    = 2
D_NOTCLOSE = 3
D_FAR      = 4
D_VERYFAR  = 5

# Направление
DIR_L  = 0
DIR_FL = 45
DIR_F  = 90
DIR_FR = 135
DIR_R  = 180
DIR_BR = 225
DIR_B  = 270
DIR_BL = 315

FORWARD   = "FORWARD"
FORWARD_L = "FORWARD_L"
LEFT      = "LEFT"
BACK_L    = "BACK_L"
BACK      = "BACK"
BACK_R    = "BACK_R"
RIGHT     = "RIGHT"
FORWARD_R = "FORWARD_R"
UNKNOWN   = "UNKNOWN"

# Эти числа определяют номера строк/столбцов матрицы переходов
D_NOT = 0   # отсутствует
D_L   = 1   # слева
D_FL  = 2   # впереди-слева
D_F   = 3   # впереди
D_FR  = 4   # ...
D_R   = 5
D_BR  = 6
D_B   = 7
D_BL  = 8
D_UNK = -1

SEARCHE = 1 # режим поиска объекта
REPEAT  = 2 # режим повторения пути
regim = SEARCHE # текущий режим

#_--------------------------------------------------------------------------------------------------------
# Автоматные процедуры
#
def fProcGoFwd():
    CurrAgent.Make(tmu.PROC_GOFWD, 0)

def fProcRandomTurn():
    if( FSM.rand(2)==0):      #random.choice(0,1)==0):        
        CurrAgent.Make(tmu.PROC_GORIGHT, 0)
    else:
        CurrAgent.Make(tmu.PROC_GOLEFT, 0)

def fProcGoBack():
    CurrAgent.Make(tmu.PROC_GOBACK, 0)

def fProcGoStop():
    CurrAgent.Make(tmu.PROC_STOP, 0)

def fProcGoRight():
    CurrAgent.Make(tmu.PROC_GORIGHT, 0)

def fProcGoLeft():
    CurrAgent.Make(tmu.PROC_GOLEFT, 0)
    
#
# Автоматные предикаты условий перехода
#
def fIsLB():
    return sGetFL(CurrAgent)<5

def fIsRB():
    return sGetFR(CurrAgent)<5

def fIsRB():
    return sGetFR(CurrAgent)<5

def fLeft(X):
    if(X[0]==D_L): return True
    else: return False
  
def fFL(X):
    if(X[0]==D_FL): return True
    else: return False
  
def fFwd(X):
    if(X[0]==D_F): return True
    else: return False
  
def fFR(X):
    if(X[0]==D_FR): return True
    else: return False
  
def fRight(X):
    if(X[0]==D_R): return True
    else: return False
  
def fHomeDist(X):
    global JobREGIM
    if(JobREGIM==DEBUG):     print "dist for object = ", X[1]
    return X[1]

def ProcFound(obj):
    global ObjFound, regim, GlobalTimer, ObjColor

    print "GlobalTimer = ", GlobalTimer
    if (ObjX[2] == ObjColor):  # нашли объект нужного цвета
        ObjFound = True
        if (regim == SEARCHE):
            regim = REPEAT

def ProcComeBack(home):
    # вернулись домой
    CurrAgent.agent.status = S_WALKING

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
    return

def sGetSR(A):
    s = A.agent.MainSensors[3]
    if(s==0): s = 100
    return s

def sGetC(A):
    s = A.agent.MainSensors[4]
    if(s==0): s = 100
    return s

def sGetColor(A):
    s = A.Dataserver[1]
    return s

def sGetLight(A):
    s = A.Dataserver[1]
    if(s==0): s = 100
    return s

# номера автоматов
fsm_obstacle    = -1
fsm_reflex      = -1
fsm_walk        = -1
fsm_bypassright = -1
fsm_bypassleft  = -1
fsm_moveto      = -1
fsm_endofpath   = -1
fsm_toleft      = -1
fsm_toright     = -1
fsm_back        = -1
fsm_aroundl     = -1
fsm_aroundr     = -1
fsm_comeback    = -1

#------------------------------------------------------------------------------
# Инициалиизация системы
def InitSystem(nodename, envfilename, mapfilename, agentfilename):
    global Agents
    global FSM, FSMList, ObjX
    global MyHome

    # Инициалиизация ROS
    rospy.init_node(nodename)
  
    # Входной топик
    # Подписываемся на топик ardans, второй параметр - объем кэша отправки
    inpqlen = len(Agents)*7
    rospy.Subscriber("/ardans_topic", tmu.ans, tmu.rsans_callback, queue_size=inpqlen)

    # Выходной топик
    outqlen = len(Agents)*7
    pub_cmd = rospy.Publisher("/actions_topic", tmu.action, queue_size=outqlen)

    # Создаем роботов
    for a in Agents:
        r = tmu.TRobot(a,pub_cmd)
        tmu.Robots.append(r)

    #---------------------------------
    # Создаем автоматы
    
    f = TAutomaton("Obstacle", ["S", "1", "B", "L", "R", "T"], ["T"], "S")
    f.SetTCnt(10)
    f.addRule(TRule("S", "B", "fIsLB() and fIsRB()", "fProcGoBack(); self.ResetT()"))
    f.addRule(TRule("S", "L", "fIsLB()",             "fProcGoRight(); self.ResetT()"))
    f.addRule(TRule("S", "R", "fIsRB()",             "fProcGoLeft(); self.ResetT()"))
    f.addRule(TRule("S", "T", "else",                "fProcGoStop(); self.ResetT()"))
    f.addRule(TRule("B", "B", "self.CheckT()", ""))
    f.addRule(TRule("B", "1", "else",                "fProcRandomTurn(); self.ResetT()"))
    f.addRule(TRule("1", "1", "self.CheckT()", ""))
    f.addRule(TRule("1", "T", "else",                "fProcGoStop(); self.ResetT()"))
    f.addRule(TRule("L", "L", "self.CheckT()", ""))
    f.addRule(TRule("L", "T", "else",                "fProcGoStop(); self.ResetT()"))
    f.addRule(TRule("R", "R", "self.CheckT()", ""))
    f.addRule(TRule("R", "T", "else",                "fProcGoStop(); self.ResetT()"))

    FSMList.append(f)

    f = TAutomaton("Reflex", ["S", "1", "2", "3", "T"], ["T"], "S")
    f.SetTCnt(10)
    f.addRule(TRule("S", "1", "fIsLB() and fIsRB()", "fProcGoBack(); self.ResetT()"))
    f.addRule(TRule("S", "1", "fIsLB()",             "fProcGoBack(); self.ResetT()"))
    f.addRule(TRule("S", "2", "fIsRB()",             "fProcGoBack(); self.ResetT()"))
    f.addRule(TRule("S", "T", "else",                "fProcGoStop(); self.ResetT()"))
    f.addRule(TRule("1", "1", "self.CheckT()", ""))
    f.addRule(TRule("1", "3", "else",                "fProcGoRight(); self.ResetT()"))
    f.addRule(TRule("2", "2", "self.CheckT()", ""))
    f.addRule(TRule("2", "3", "else",                "fProcGoLeft(); self.ResetT()"))
    f.addRule(TRule("3", "3", "self.CheckT()", ""))
    f.addRule(TRule("3", "T", "else",                "fProcGoStop(); self.ResetT()"))

    FSMList.append(f)
    
    f = TAutomaton("Walk", ["S", "1", "2", "3"], ["T"], "S")  # почему состояние Т не входит в список состояний?
    f.Trace = False
    f.SetTCnt(10)
    f.addRule(TRule("S", "1", "True",          "fProcGoFwd(); self.ResetT()"))
    f.addRule(TRule("1", "1", "self.CheckT()", "fProcGoFwd()"))
    f.addRule(TRule("1", "2", "else",          "fProcRandomTurn(); self.ResetT()"))
    f.addRule(TRule("2", "2", "self.CheckT()", ""))
    f.addRule(TRule("2", "3", "else",          "fProcGoFwd(); self.ResetT()"))
    f.addRule(TRule("3", "3", "self.CheckT()", ""))
    f.addRule(TRule("3", "S", "else",          ""))    

    FSMList.append(f)

    f = TAutomaton("ToRight", ["S", "R", "T"], ["T"], "S")
    f.SetTCnt(20)  # количество тактов, в течение которых робот поворачивает
    f.addRule(TRule("S", "R", "True",          "fProcGoRight(); self.ResetT()"))
    f.addRule(TRule("R", "R", "self.CheckT()", ""))
    f.addRule(TRule("R", "T", "else",          "fProcGoFwd()"))
    
    FSMList.append(f)

    f = TAutomaton("ToLeft", ["S", "L", "T"], ["T"], "S")
    f.SetTCnt(20)  # количество тактов, в течение которых робот поворачивает
    f.addRule(TRule("S", "L", "True",          "fProcGoLeft(); self.ResetT()"))
    f.addRule(TRule("L", "L", "self.CheckT()", ""))
    f.addRule(TRule("L", "T", "else",          "fProcGoFwd()"))
    
    FSMList.append(f)

    f = TAutomaton("BypassLeft", ["S", "L", "F", "T"], ["T"], "S")
    f.SetTCnt(10)
    f.Trace = True
    f.addRule(TRule("S", "L", "fLeft(ObjX) or fFL(ObjX) or fFwd(ObjX)", "fProcGoLeft()"))
    f.addRule(TRule("L", "L", "fLeft(ObjX) or fFL(ObjX) or fFwd(ObjX)", ""))
    f.addRule(TRule("L", "F", "else",                                   "fProcGoFwd()"))
    f.addRule(TRule("S", "F", "else",                                   "fProcGoFwd()"))
    f.addRule(TRule("F", "F", "fRight(ObjX) or fFR(ObjX)",              "fProcGoFwd()"))
    f.addRule(TRule("F", "L", "fLeft(ObjX) or fFL(ObjX) or fFwd(ObjX)", "fProcGoLeft()"))
    f.addRule(TRule("F", "T", "else",                                   "fProcGoStop()"))

    FSMList.append(f)

    f = TAutomaton("BypassRight", ["S", "R", "F", "T"], ["T"], "S")
    f.SetTCnt(10)
    f.Trace = True
    f.addRule(TRule("S", "R", "fRight(ObjX) or fFR(ObjX) or fFwd(ObjX)", "fProcGoRight()"))
    f.addRule(TRule("R", "R", "fRight(ObjX) or fFR(ObjX) or fFwd(ObjX)", ""))
    f.addRule(TRule("R", "F", "else",                                    "fProcGoFwd()"))
    f.addRule(TRule("S", "F", "else",                                    "fProcGoFwd()"))

    f.addRule(TRule("F", "F", "fLeft(ObjX) or fFL(ObjX)",                "fProcGoFwd()"))
    f.addRule(TRule("F", "R", "fRight(ObjX) or fFR(ObjX) or fFwd(ObjX)", "fProcGoRight()"))
    f.addRule(TRule("F", "T", "else",                                    "fProcGoStop()"))

    FSMList.append(f)

    f = TAutomaton("MoveTo", ["S", "A", "L", "R", "F", "T"], ["T"], "S")
    f.SetTCnt(10)
    f.Trace = True
    f.addRule(TRule("S", "A", "fDist(ObjX) > 1",                "fProcGoStop()"))
    f.addRule(TRule("S", "T", "else",                           "fProcGoStop(); ProcFound(ObjX)"))

    f.addRule(TRule("A", "L", "fLeft(ObjX) or fFL(ObjX)",       "fProcGoLeft()"))
    f.addRule(TRule("L", "L", "fLeft(ObjX) or fFL(ObjX)",       ""))
    f.addRule(TRule("L", "F", "else",                           "fProcGoFwd()"))

    f.addRule(TRule("A", "R", "fRight(ObjX) or fFR(ObjX)",      "fProcGoRight()"))
    f.addRule(TRule("R", "R", "fRight(ObjX) or fFR(ObjX)",      ""))
    f.addRule(TRule("R", "F", "else",                           "fProcGoFwd()"))
    
    f.addRule(TRule("A", "F", "fFwd(ObjX)",                     "fProcGoFwd()"))
    f.addRule(TRule("A", "A", "else",                           "fProcGoBack()"))
    f.addRule(TRule("F", "F", "fDist(ObjX) > 1 and fFwd(ObjX)", ""))
    f.addRule(TRule("F", "L", "fFL(ObjX)",                      "fProcGoLeft()"))
    f.addRule(TRule("F", "R", "fFR(ObjX)",                      "fProcGoRight()"))
    f.addRule(TRule("F", "T", "else",                           "fProcGoStop(); ProcFound(ObjX)"))    

    FSMList.append(f)

    f = TAutomaton("ComeBack", ["S", "A", "T"], ["T"], "S")
    f.SetTCnt(10)
    f.Trace = False
    f.addRule(TRule("S", "A", "fHomeDist(MyHome) > DIST_1",     "fProcGoFwd()"))
    f.addRule(TRule("S", "T", "else",                           "fProcGoFwd(); ProcComeBack(MyHome)"))
    f.addRule(TRule("A", "A", "fHomeDist(MyHome) > DIST_1",     "fProcGoFwd()"))
    f.addRule(TRule("A", "T", "else",                           "fProcGoFwd(); ProcComeBack(MyHome)"))

    FSMList.append(f)

    f = TAutomaton("EndOfPath", ["S", "T"], ["T"], "S")
    f.SetTCnt(10)
    f.Trace = True
    f.addRule(TRule("S", "T", "True",  "fProcGoStop()"))

    FSMList.append(f)

    f = TAutomaton("Back", ["S", "T"], ["T"], "S")
    f.SetTCnt(10)
    f.Trace = True # False
    f.addRule(TRule("S", "T", "True",  "fProcGoBack()"))

    FSMList.append(f)
    
    f = TAutomaton("AroundL", ["S", "L", "F", "T"], ["T"], "S")
    f.SetTCnt(6)  # количество тактов, в течение которых робот поворачивает или идет прямо
    f.Trace = True
    f.addRule(TRule("S", "L", "True",          "fProcGoLeft(); self.ResetT()"))
    f.addRule(TRule("L", "L", "self.CheckT()", ""))
    f.addRule(TRule("L", "F", "else",          "fProcGoFwd(); self.ResetT()"))
    f.addRule(TRule("F", "F", "self.CheckT()", ""))
    f.addRule(TRule("F", "L", "else",          "fProcGoLeft(); self.ResetT()"))
    
    FSMList.append(f)

    f = TAutomaton("AroundR", ["S", "R", "F", "T"], ["T"], "S")
    f.Trace = True
    f.SetTCnt(6)  # количество тактов, в течение которых робот поворачивает или идет прямо
    f.addRule(TRule("S", "R", "True",          "fProcGoRight(); self.ResetT()"))
    f.addRule(TRule("R", "R", "self.CheckT()", ""))
    f.addRule(TRule("R", "F", "else",          "fProcGoFwd(); self.ResetT()"))
    f.addRule(TRule("F", "F", "self.CheckT()", ""))
    f.addRule(TRule("F", "R", "else",          "fProcGoRight(); self.ResetT()"))
    
    FSMList.append(f)

def FindFSM(name):
    global FSMList
    for f in FSMList:
        if f.Name==name:
            return f
    gdic.error("FSM "+name+" not found")

################################################################################
# Service functions
################################################################################

def Distance(n):
    if(n<=DIST_1): return D_NEAR
    if(n<=DIST_2): return D_CLOSE
    if(n<=DIST_3): return D_NOTCLOSE
    if(n<=DIST_4): return D_FAR
    return D_VERYFAR
  
def Direction(n):
#  слева - 0, справа - 180
    if(n<=(DIR_L+22)  or  n>(DIR_BL+22)):  return D_L
    if(n>=(DIR_FL-22) and n<=(DIR_FL+22)): return D_FL
    if(n>=(DIR_F-22)  and n<=(DIR_F+22)):  return D_F
    if(n>=(DIR_FR-22) and n<=(DIR_FR+22)): return D_FR
    if(n>=(DIR_R-22)  and n<=(DIR_BR+22)): return D_R
    if(n>=(DIR_BR-22) and n<=(DIR_BR+22)): return D_BR
    if(n>=(DIR_B-22)  and n<=(DIR_B+22)):  return D_B
    if(n>=(DIR_BL-22) and n<=(DIR_BL+22)): return D_BL
    return D_UNK

def ToTurn(n):
    if(n<68):  return -45
    if(n>112): return 45 
    return 0

def ToObject(n):  
    if(n>=80 and n<=100):  return 0
    if(n>90):  return 1
    if(n<90):  return -1
    #return n-90
    
# фильтрация данных суперлокатора (SL); 
# режим = 0 - возвращает необработанные данные, режим = 1 - приведенные к линвистическим переменным
def cLocator(SL, reg):
    loc = []
    flag = False
    ji = 0
    j2 = 0
    
    for i in range(0, len(SL)):
        point = []
        if(SL[i][1]>0):
            if(not flag):  
                j1 = i
                flag = True
            elif(SL[i][1]<>SL[j1][1]):
                j2 = (i - 1 + j1)/2
                if(reg==0):
                    point.append(j2)            # direction
                    point.append(SL[j2][0])     # distance
                else:
                    point.append(Direction(j2))        # direction
                    point.append(Distance(SL[j2][0]))  # distance
                point.append(SL[j2][1])                # color
                loc.append(point)
                j1 = i # новый объект
        if(SL[i][1]==0 and flag):
            j2 = (i - 1 + j1)/2
            if(reg==0):
                point.append(j2)            # direction
                point.append(SL[j2][0])     # distance
            else:
                point.append(Direction(j2))        # direction
                point.append(Distance(SL[j2][0]))  # distance
            point.append(SL[j2][1]) # color
            loc.append(point)
            flag = False
    return loc
  
def TMU(n):
    if(n==D_F): return tmu.PROC_GOFWD
    if(n==D_B): return tmu.PROC_GOBACK
    if(n==D_L): return tmu.PROC_GOLEFT
    if(n==D_R): return tmu.PROC_GORIGHT
    return tmu.PROC_STOP
 
"""
# 0 номер точки
# 1 direction
# 2 distance
# 3 color
# 4 направление на ориентир перед тем, как он пропал из поля зрения
# 5 расстояние до ориентира перед тем, как он пропал из поля зрения
# 6 номер отрезка (последнего), на котором наблюдался ориентир
# 7 пустое поле
"""
def sort_col(i):
    return i[0]

def setFSM(name):
    if  (name == "BypassRight"):  return fsm_bypassright
    elif(name == "BypassLeft"):   return fsm_bypassleft
    elif(name == "ToLeft"):       return fsm_toleft
    elif(name == "ToRight"):      return fsm_toright
    elif(name == "MoveTo"):       return fsm_moveto
    elif(name == "AroundL"):      return fsm_aroundl
    elif(name == "AroundR"):      return fsm_aroundr
    elif(name == "ComeBack"):     return fsm_comeback
    elif(name == "Back"):         return fsm_back
    elif(name == "EndOfPath"):    return fsm_endofpath
    gdic.error("UNKNOWN automat "+name)
    return None

def ADist(x1, y1, x2, y2):   # определяет расстояние между двумя точками с учетом топологии тора
    global Env
    d1 = abs(x1-x2)
    d2 = abs(y1-y2)
#    print "ADist:", x1, y1, x2, y2
#    print "d1, d2:", d1, d2
    if Env.UseTorusRegime:
        dx = min( d1, (Env.DIM_X-d1) )
        dy = min( d2, (Env.DIM_Y-d2) )
    else: 
        dx = d1
        dy = d2
    a = math.sqrt( dx**2 + dy**2 )
#    print "dx, dy, a:", dx, dy, a
#    #gdic.pause("Enter")
    return a
  
# Возвращает список соседей, увиденных локатором (без повторов)
def AgentList(pl):
    cr = 0
    flag = False
    vect = []
    # сортировка данных локатора
    # pl.sort(key=sort_col, reverse=True)
    
    for i in range(0, len(pl)):
        if(pl[i] > 0):
            if pl[i] <> cr:
                vect.append(pl[i])
                cr = pl[i]
    return vect

def AFind(id):
    global Agents
    for a in Agents:
        if(a.id == id):
            return a
    return None
  
# возвращает True, если у агента aid уровень агрессивности al меньше, чем уровень хотя бы одного соседа из списка nb
def notLeader(aid, al, nb):
    #n = 0
    #k = len(nb)-1
    #if nb[0] == nb[k]: n = 1 # пропускаем первый элемент, если он есть в конце списка
    for i in range(0, len(nb)):
        if aid//RAZMER <> nb[i]//RAZMER: 
            b = AFind(nb[i])
            if not(b is None): 
                if(JobREGIM==DEBUG): print "#####", aid, " -- ", nb[i], " -- ", al, " -- ", b.alevel
                if al < b.alevel: return True
    return False
  
def XYtoDD(A):
    obj_home = [ 0, 10, 1 ]
    xh = A.agent.ahome[0]
    yh = A.agent.ahome[1]
    color_h = A.agent.id//RAZMER
    xa = A.agent.pos[0]
    ya = A.agent.pos[1]
    return obj_home

# Определение направления на дом для агента А
def Angle(A):
    xh = A.agent.ahome[0]
    yh = A.agent.ahome[1]
    xa = A.agent.pos[0]
    ya = A.agent.pos[1]
    if (xh == xa and yh > ya): return 90
    if (xh == xa and yh < ya): return 270
    if (yh == ya and xh > xa): return 0
    if (yh == ya and xh < xa): return 180
    if (xh == xa): return 0
    n = abs(yh-ya)/abs(xh-xa)
    fi = math.atan(n)*180/3.1415
    if (ya > yh and xa > xh): return 180+fi
    if (ya > yh and xa < xh): return 360-fi
    if (xa > xh and ya < yh): return 180-fi
    if (xa < xh and ya < yh): return fi
    
################################################################################
# MAIN
################################################################################

def main(envfilename, mapfilename, agentfilename):
    global Env, Agents, GlobalTimer
    global FSM, CurrAgent
    global regim, MyHome
    global fsm_obstacle, fsm_reflex, fsm_walk, fsm_bypassright, fsm_bypassleft, fsm_moveto, fsm_endofpath, fsm_back, fsm_aroundl, fsm_aroundr, fsm_toleft, fsm_toright, fsm_comeback
    global JobREGIM

    SD =   [ 0, 0, 0, 0, 0, 0, 0, 0, 0 ] # массив для хранения статистики
    stat = [ 0, 0, 0, 0, 0, 0, 0, 0, 0 ] # массив для хранения промежуточной статистики (за такт)
    Fgr =  [ 0, 0, 0, 0, 0 ] # расстояние до найденной еды
      
    # Инициалиизация системы
    print "Init environment... "
    Env = TEnv(envfilename)

    print "Init map", mapfilename, "..."
    exec (open(mapfilename).read())

    print "Init agents", agentfilename, "..."
    exec (open(agentfilename).read())

    InitSystem('swhunt2p', envfilename, mapfilename, agentfilename)

    print "Start main loop"
    r = rospy.Rate(50) # 50hz

    ############################################################################
    # Инициалиизация системы моделирования
    ############################################################################

    global LenRoute   # количество отрезков на маршруте
    global StartLine  # начало текущего отрезка
    num = 0
    m = 0
    StartLine = 0
   
    global RAZMER, DIST_4, FoodColor1, FoodColor2
 
    #####################################################
    # Основной цикл
    #####################################################

    fsm_obstacle = FindFSM("Obstacle")
    fsm_reflex = FindFSM("Reflex")
    fsm_walk = FindFSM("Walk")
    fsm_bypassright = FindFSM("BypassRight")
    fsm_bypassleft = FindFSM("BypassLeft")
    fsm_toright = FindFSM("ToRight")
    fsm_toleft = FindFSM("ToLeft")
    fsm_moveto = FindFSM("MoveTo")
    fsm_back = FindFSM("Back")
    fsm_comeback = FindFSM("ComeBack")
    fsm_endofpath = FindFSM("EndOfPath")
    fsm_aroundl = FindFSM("AroundL")
    fsm_aroundr = FindFSM("AroundR")
    FSM = fsm_walk
    #-------------------------------------------------
    prevAuto = -1 # предыдущий автомат
    prevObj  = -1 # предыдущий ориентир
    eoj = False
    
    xy1 = [ 0, 0, 0 ]
    xy2 = [ 0, 0, 0 ]
    NUM_i = [ 0, 0, 0, 0, 0 ] # массив для хранения количества агентов разных групп
    
    # Перемешать агентов
    #"""
    cnt = len(tmu.Robots)
    for i in range(0, 50):
        n = random.randint(0, cnt-1)
        m = random.randint(0, cnt-1)
        b = tmu.Robots[n]
        tmu.Robots[n] = tmu.Robots[m]
        tmu.Robots[m] = b
    #"""
    # ввести для каждого агента дополнительные поля
    for a in tmu.Robots:
        a.agent.alevel = 0.0        # уровень агрессивности
        a.agent.alimit = 0.5        # порог срабатывания
        a.agent.atime  = 0          # время неагрессивного поведения
        a.agent.status = S_WALKING  # текущее состояние агента
        # агент запоминает свой дом - это та точка, в которой он появился впервые (x, y, color)
        a.agent.ahome = a.agent.pos # координаты дома (x,y)
        a.agent.ahome[2] = a.agent.id//RAZMER  # цвет дома
        a.agent.adist = 0           # сумма расстояний от дома
        a.agent.adamage = 0         # количество поражений
        a.agent.pp_my = 0           # количество приемов своей пищи
        a.agent.pp_notmy = 0        # количество приемов чужой пищи
        m = a.agent.id//RAZMER
        NUM_i[m] +=1
    
    # Максимальное количество групп роботов - 4, минимальное - 2.
    # Если их меньше 4-х, NUM_i[3] и NUM_i[4] надо установить в 1, т.к. на них делят
    if NUM_i[3] == 0: NUM_i[3]=1
    if NUM_i[4] == 0: NUM_i[4]=1
    
    while not rospy.is_shutdown():
        if (GlobalTimer>MAXSTEPS or eoj): break
        for a in tmu.Robots:
            a.RequestAllSensors(immediate = False, req_main_sensors=True, req_locator=True, req_super_locator=True, req_i2cdata=True, req_tsoprc5=False, req_registers=False)
            #a.ShowStatus(show_main_sensors=True, show_locator=False, show_super_locator=False, show_dataserver=True, show_tsoprc5=False, show_registers=False)
            # Определить текущее местоположение
            ppp = len(a.agent.MainSensors)-1
            a.agent.pos = [ a.agent.MainSensors[ppp-2], a.agent.MainSensors[ppp-1], a.agent.MainSensors[ppp] ]
            
            pointlist = []
            splist = []
            plcolor = 0
            CurrAgent = a
            
            pointlist = a.agent.Locator   # видит соседей
            #if(JobREGIM==DEBUG):  print "Локотор соседей", a.agent.id, ": ", pointlist
            #splist = cLocator(a.agent.SuperLocator, 0)  # видит еду
            #if(JobREGIM==DEBUG): print "Суперлокатор: id = ", a.agent.id, ", ", splist                
            # Определить расстояние до дома
            S = ADist(a.agent.ahome[0], a.agent.ahome[1], a.agent.pos[0], a.agent.pos[1])
            a.agent.adist += S # сумма расстояний от дома
                   
            MyHome = [ 0, S//1, a.agent.ahome[2] ] # дом как объект: направление, расстояние, цвет 
            #print a.agent.id, "MyHome = ", MyHome, a.agent.status
            #-----------------------------
            # Логика
            #
            # Нашел еду - питается
            plcolor = sGetColor(a)  # видит еду
            if (plcolor == FoodColor1 or plcolor == FoodColor2):
                a.EatProc()
                a.agent.status = S_FORAGING # нашел еду - должен отнести ее в муравейник
                # Определяем направление на муравейник и устанавливаем угол поворота на него
                alfa =  Angle(a)
                tmu.SetAgentPos(a.agent.id, a.agent.pos[0], a.agent.pos[1], alfa )
                m = a.agent.id//RAZMER
                Fgr[m] += S   # суммировать расстояние от дома до еды по группам
                if (a.agent.id//RAZMER) == plcolor: a.agent.pp_my += 1
                else: a.agent.pp_notmy += 1
                if(JobREGIM==DEBUG): print "Датчик еды", a.agent.id, ": ", plcolor, " my - ", a.agent.pp_my, " notmy - ", a.agent.pp_notmy

            # Препятствие
            if(sGetFL(a)<5 or sGetFR(a)<5):
                if(JobREGIM==DEBUG): print "Obstacle"
                # вместо автомата - принудительный поворот на 180 градусов: не успевает отработать поворот при отходе от препятствия
                tmu.SetAgentPos(a.agent.id, a.agent.pos[0], a.agent.pos[1], a.agent.pos[2]+180 )
                #if(FSM!=fsm_obstacle):
                #    if(JobREGIM==DEBUG): print "load fsm_obstacle"
                #    FSM = fsm_obstacle
                #    FSM.reset()                
                
            res = FSM.step() # автоматически считает такты - self.T++
                                
            # Фуражирование (движение к муравейнику)
            if(a.agent.status == S_FORAGING):
                if(FSM!=fsm_comeback):
                    if(JobREGIM==DEBUG): print "load fsm_comeback"
                    FSM = fsm_comeback
                    FSM.reset()                
                
            res = FSM.step() # автоматически считает такты - self.T++
                                
            if res==FSM_FINISHED:
                if(JobREGIM==DEBUG): print "load fsm_walk"
                FSM = fsm_walk
                FSM.reset()
                
            a.agent.atime += 1  # увеличиваем время без агрессии
                
            if(JobREGIM==DEBUG): 
                print a.agent.id, " S = ", S, " pos =", a.agent.pos, " ahome=", a.agent.ahome
            m = a.agent.id//RAZMER
            stat[m] += S   # суммировать расстояние по группам

            if a.agent.id == 100: 
                #if(JobREGIM==DEBUG): 
                if(JobREGIM==DEBUG): print stat[1]//NUM_i[1], stat[2]//NUM_i[2], stat[3]//NUM_i[3], stat[4]//NUM_i[4]
                for i in range(0, len(stat)):
                    SD[i] += stat[i]  # суммировать расстояние по группам
                    stat[i] = 0
                   
            # Определить новый уровень агрессивности
            if (Arule == ONLYDIST): 
                a.agent.alevel = min (1, 1.0/(S/5+1))  # только расстояние
                #if Distance(S)==D_VERYFAR: a.agent.alevel = 0
                #else: a.agent.alevel = min (1, 1.0/(S/5+1))  # только расстояние
                #if(JobREGIM==DEBUG): print a.agent.id, " S = ", S, " alevel=", a.agent.alevel
            if (Arule == TIMEDIST): 
                a.agent.alevel = min (1, 1.0/(S/5+1)+a.agent.atime*0.01)  # расстояние и время

            if(JobREGIM==DEBUG): print a.agent.id, " S = ", S, " alevel=", a.agent.alevel, " a.agent.atime=", a.agent.atime
            #a.agent.alevel = min(1, 0.6*(1/(S+1)) + 0.4*(a.agent.atime/100.0))
                
            # Посмотреть на соседей и сравнить уровни агрессивности
            #if(JobREGIM==DEBUG): 
            """
            neighbour = AgentList(pointlist) 
            if(JobREGIM==DEBUG): print "neighbour: ", neighbour
            if notLeader(a.agent.id, a.agent.alevel, neighbour):
                if(JobREGIM==DEBUG): 
                    print "^^^^^^^^^", a.agent.id, " go to home ", a.agent.ahome
                tmu.SetAgentPos(a.agent.id, a.agent.ahome[0], a.agent.ahome[1], a.agent.pos[2]+FSM.rand(40) )
                a.agent.atime = 0  # обнуляем время без агрессии
                a.agent.adamage += 1 # количество "поражений"
            """    
            if (JobREGIM==DEBUG): print "agent ",a.agent.id,", ",a.agent.alevel,", ", S 
            # Конец логики
            #-----------------------------

            if a.agent.id == RAZMER: GlobalTimer += 1
            #print a.agent.id//RAZMER, a.agent.atime, a.agent.pos

            r.sleep()

    # вывод данных по группам (номер группы равен первой цифре идентификатора робота)
    #print "id Приемов своей пищи, Приемов чужой пищи , среднее расстояние, количество поражений"
    for i in range(1,4):
        my = 1
        nm = 1
        for a in tmu.Robots:
            if a.agent.id//RAZMER == i: 
                my += a.agent.pp_my
                nm += a.agent.pp_notmy
                #print a.agent.id, a.agent.pp_my, a.agent.pp_notmy, a.agent.adist//MAXSTEPS, a.agent.adamage
        print my, nm, Fgr[i], Fgr[i]//(my+nm)
        
    print " SD = ", SD[1]//MAXSTEPS, SD[2]//MAXSTEPS

    gdic.terminate_program()

################################################################################
#
################################################################################
if __name__ == '__main__':

    if (len(sys.argv) < 4):
        print "\n", Title, "\n\nUsage is:", sys.argv[0], "envfile mapfile agentfile"
        sys.exit(1)

    envfile = sys.argv[1]
    mapfile = sys.argv[2]
    agentfile = sys.argv[3]

    main(envfile, mapfile, agentfile)
