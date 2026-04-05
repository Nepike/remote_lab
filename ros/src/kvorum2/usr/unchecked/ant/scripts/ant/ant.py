#!/usr/bin/env python
# coding: utf-8
"""
  ant.py

  06.02.15
  Version 1.08
  LP 06.07.2016

"""

import os, sys, time

sys.path.append("../../pylib")
sys.path.append("src/kvorum/pylib")

import roslib, rospy, copy, random

import gdic, rcproto, tmurobot as tmu

# Подгружаем модуль именно таким образом, чтобы не было проблем с видимостью
exec (open("src/kvorum/pylib/fsm.py").read())

from env import TEnv
from agent import TAgent, TSensor
###################################################################

Title = "Ant 1.06"

# Задание начального значения random (чтобы повторял путь)
random.seed(161)  #  199 (90)- в разные стороны) #101+) 11-) #38+) # 66-) # 75+) # 101-) # 26+

GlobalTimer = 0
Env = None
Agents = []
CurrAgent = None ###?

# Глобалы
DEBUG = "DEBUG"
WORK  = "WORK"    # режим работы: печатает только данные о путях

JobREGIM = DEBUG  # режим отладки: печатает все

# начальное положение роботов
XBegin = 45
YBegin = 15

# Текущий автомат, список автоматов
FSM = None
FSMList = []
ObjX = [ 0, 0, 0 ] # текущий ориентир (направление, расстояние до ориентира, цвет)
ObjY = [ 0, 0, 0 ]
CurrPoint = -1  # текущая точка на пути

ObjFound = False  # признак того, что первый робот нашел объект

# Маршрут
Route = []     # сам маршрут
LenRoute = 0   # количество отрезков на маршруте
StartLine = 0  # начало текущего отрезка
#NumStep = 0   # номер пройденного шага при повторе
ObjColor = 1   # цвет искомого объекта
Way = []       # путь для второго робота

# Расстояние: вплотную, близко, не далеко/не близко, далеко, очень далеко (> DIST_4)
DIST_1  = 2
DIST_2  = 5
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
  
def fDist(X):
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
    
# Эти функции пришлось перенести из agents.ctl
def sGetFL(A):
    global JobREGIM
    #if(JobREGIM==DEBUG):     print "sGetFL(A)"
    s = A.agent.MainSensors[0]
    if(s==0): s = 100
    return s

def sGetFR(A):
    global JobREGIM
    #if(JobREGIM==DEBUG):     print "sGetFR(A)"
    s = A.agent.MainSensors[1]
    if(s==0): s = 100
    return s

# номера автоматов
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

#------------------------------------------------------------------------------
# Инициалиизация системы
def InitSystem(nodename, envfilename, mapfilename, agentfilename):
    global Agents
    global FSM, FSMList, ObjX

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

# Проверяет, нужно ли добавлять отрезок в маршрут.
def AppendRoute(ln):
    global LenRoute   # количество отрезков на маршруте
    global StartLine  # начало текущего отрезка
    global JobREGIM
    
    cnt = 0  # счетчик количества одинаковвых точек на отрезке
    cntRoute = len(Route)  # длина маршрута (количество точек на всех отрезках)
    cntLine = len(ln)      # длина отрезка (количество точек)
    if(JobREGIM==DEBUG): print "AppendRoute: длина отрезка=", cntLine, ", длина маршрута = ", cntRoute

    if(cntRoute==0):   # добавляем первый отрезок
        StartLine = 0
        LenRoute = 1
        for n in range(0, cntLine):
            ln[n][0] = LenRoute
            Route.append(ln[n])
    else:
        # Проверяем, изменились ли ориентиры с предыдущего отрезка  
        for i in range(0, cntLine):
            for j in range(StartLine, cntRoute):
                if(Route[j][1]==ln[i][1] and Route[j][2]==ln[i][2] and Route[j][3]==ln[i][3]): cnt = cnt + 1
            
        if(cnt!=(cntRoute-StartLine) or cnt!=cntLine): # не все точки нашли или в новом отрезке точек больше
            StartLine = cntRoute
            LenRoute = LenRoute + 1
            for n in range(0, cntLine):
                ln[n][0] = LenRoute
                Route.append(ln[n])

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
    return i[6]

def RouteProc(route):
    global ObjColor

    RT = []
    cnt = 0
    for r in route:
        if(cnt==0):  # точек нет, добавляем
            cnt += 1
            RT.append([ cnt, r[1], r[2], r[3], r[1], r[2], r[0], 0 ])
        else:
            found = False
            for t in RT:       # ищем точку такого же цвета
                if (t[3]==r[3]):   # тот же цвет
                    if ((r[0]-t[6])==1):  # ориентир был на предыдущем отрезке
                        t[4]=r[1]         # добавляем ей направление,
                        t[5]=r[2]         # расстояние
                        t[6]=r[0]         # и номер отрезка
                        found = True      # такой ориентир найден
            if (not found):  # не найден
                cnt += 1     # считаем номер точки
                RT.append([cnt, r[1], r[2], r[3], r[1], r[2], r[0], 0 ])
                       
    RT.sort(key=sort_col)
    return RT

# удаляем из маршрута для второго робота точки после искомого объекта
def WayProc(route):
    global ObjColor
    way = []
    
    RT = RouteProc(route)
    find = False
    for r in RT:
        if(find): break
        if (r[3] == ObjColor):  
            r[4] = 3      # к искомому объекту всегда надо двигаться прямо!
            find = True
        way.append(r)
    print "Путь для второго робота: ", way
    return way

def RoadProc(road):
    RT = []
    pnt = []
    pnt.append([road[0][0], road[0][1], road[0][2], road[0][3], road[0][4], road[0][5], road[0][6], road[0][7]])
    RT.append([road[0][0], road[0][1], road[0][2], road[0][3], road[0][4], road[0][5], road[0][6], road[0][7]])
    n = 0
    
    for i in range(1, len(road)):
        if (RT[n][3] <> road[i][3] or RT[n][4] <> road[i][4]): # другой цвет или другое направление обхода
            RT.append([road[i][0], road[i][1], road[i][2], road[i][3], road[i][4], road[i][5], road[i][6], road[i][7]])
            n += 1
    return RT
            
# выбор следующего автомата: возвращает имя автомата и цвет ориентира
# Параметр - номер точки в пути (Way)
def NextAuto(cp):
    global Way
    
    if(cp >= len(Way)):     return "EndOfPath",   0
    if(Way[cp][4] == D_L):  return "BypassRight", Way[cp][3]
    if(Way[cp][4] == D_R):  return "BypassLeft",  Way[cp][3]
    if(Way[cp][4] == D_FL): return "BypassRight", Way[cp][3]
    if(Way[cp][4] == D_FR): return "BypassLeft",  Way[cp][3]
    return "MoveTo", Way[cp][3]

# поиск объекта указанного цвета среди видимых роботу в данный момент
def FindObj(clr, pnt):   
    for p in pnt:
        if(p[2] == clr):  
            direct = p[0]
            dist = p[1]
            return [ direct, dist, clr ]
    return [ 0, 0, clr ]

def FindNextStep(cp, pnt):
    global Way, ObjColor
    j = -1
    for i in range(cp, len(Way)):
        for p in pnt:
            if(p[2] == Way[i][3]):
                if (p[2] == ObjColor): return i
	        else: j = i  # пытаемся найти самую последнюю точку на маршруте среди видимых
    return j

def setFSM(name):
    if  (name == "BypassRight"):  return fsm_bypassright
    elif(name == "BypassLeft"):   return fsm_bypassleft
    elif(name == "ToLeft"):       return fsm_toleft
    elif(name == "ToRight"):      return fsm_toright
    elif(name == "MoveTo"):       return fsm_moveto
    elif(name == "AroundL"):      return fsm_aroundl
    elif(name == "AroundR"):      return fsm_aroundr
    elif(name == "Back"):         return fsm_back
    elif(name == "EndOfPath"):    return fsm_endofpath
    gdic.error("UNKNOWN automat "+name)
    return None
  
def DirNextPoint(cp, obj):  # определяет направление поворота при потере ориентира
# obj - ориентир, который нужно найти. Ищет его в пути и смотрит, с какой стороны его надо обойти
    global Way #,LEFT, RIGHT
    
    for i in range(cp, len(Way)):
        if (Way[i][3] == obj[2]):
            if (Way[i][4] == D_R): return RIGHT
            if (Way[i][4] == D_L): return LEFT
    return RIGHT    
    
################################################################################
# MAIN
################################################################################

def main(envfilename, mapfilename, agentfilename):
    global NumStep, Way
    global ObjX, ObjY    # текущий ориентир
    global Env, Agents, GlobalTimer
    global FSM, CurrAgent
    global CurrPoint # текущая точка на пути
    global ObjFound, regim
    global fsm_reflex, fsm_walk, fsm_bypassright, fsm_bypassleft, fsm_moveto, fsm_endofpath, fsm_back, fsm_aroundl, fsm_aroundr, fsm_toleft, fsm_toright
    global JobREGIM
    global XBegin, YBegin
  
    # Инициалиизация системы
    print "Init environment... "
    Env = TEnv(envfilename)

    print "Init map", mapfilename, "..."
    exec (open(mapfilename).read())

    print "Init agents", agentfilename, "..."
    exec (open(agentfilename).read())

    InitSystem('swhunt2p', envfilename, mapfilename, agentfilename)

    print "Start main loop"
    r = rospy.Rate(10) # 50hz

    ############################################################################
    # Инициалиизация системы моделирования
    ############################################################################

    global LenRoute   # количество отрезков на маршруте
    global StartLine  # начало текущего отрезка
    num = 0
    StartLine = 0
    # tmu.SetAgentPos(1, XBegin, YBegin, 90)
      
    #####################################################
    # Основной цикл
    #####################################################

    fsm_reflex = FindFSM("Reflex")
    fsm_walk = FindFSM("Walk")
    fsm_bypassright = FindFSM("BypassRight")
    fsm_bypassleft = FindFSM("BypassLeft")
    fsm_toright = FindFSM("ToRight")
    fsm_toleft = FindFSM("ToLeft")
    fsm_moveto = FindFSM("MoveTo")
    fsm_back = FindFSM("Back")
    fsm_endofpath = FindFSM("EndOfPath")
    fsm_aroundl = FindFSM("AroundL")
    fsm_aroundr = FindFSM("AroundR")
    FSM = fsm_walk
    #-------------------------------------------------
    prevAuto = -1 # предыдущий автомат
    prevObj  = -1 # предыдущий ориентир
    eoj = False
    
    while not rospy.is_shutdown():
        if (GlobalTimer>1000 or eoj): break
        for a in tmu.Robots:
            a.RequestAllSensors(True)
            #a.ShowStatus()
            
            #-----------------------------
            # Логика
            #
            act = tmu.PROC_GOFWD
            arg = 0
            flag = False
            pointlist = []
            CurrAgent = a
            
            if(a.agent.id == 1 and regim==SEARCHE):  # фильтрация данных локатора без обработки (режим 0)
                pointlist = cLocator(a.agent.SuperLocator, 0)
                if(JobREGIM==DEBUG): print "Локатор: id = ", a.agent.id, ", ", pointlist		

                # Рефлекс
                if(sGetFL(a)<5 or sGetFR(a)<5):
                    if(JobREGIM==DEBUG): print "Reflex"
                    if(FSM!=fsm_reflex):
                        if(JobREGIM==DEBUG): print "load fsm_reflex"
                        FSM = fsm_reflex
                        FSM.reset()                
                
                res = FSM.step() # автоматически считает такты - self.T++
                                
                if res==FSM_FINISHED and not ObjFound:
                    if(JobREGIM==DEBUG): print "load fsm_walk"
                    FSM = fsm_walk
                    FSM.reset()
                
                # ориентиры
                flag = False
                newline = []
                #ugol = 0

                for i in range(0, len(pointlist)):
                    if(pointlist[i][2]==ObjColor):
                        ObjX = [ Direction(pointlist[i][0]), Distance(pointlist[i][1]), ObjColor ]
                        if(JobREGIM==DEBUG): print "ObjX = ", ObjX
                        flag = True  # нашли объект                        
                    if(pointlist[i][1]>0):
                        row = []
                        row.append(LenRoute)                   # 0 номер отрезка маршрута
                        row.append(Direction(pointlist[i][0])) # 1 direction
                        row.append(Distance(pointlist[i][1]))  # 2 distance
                        row.append(pointlist[i][2])            # 3 color
                        # пробуем добавить точку в отрезок
                        num = len(newline)
                        if(num==0):
                            newline.append(row)  # добавляем, если отрезок пуст
                        elif(newline[num-1][1]!=row[1] or newline[num-1][3]!=row[3]):  # newline[num-1][2]!=row[2] or 
                            newline.append(row)  # добавляем, если есть отличия в точках
                    
                if (flag): 
                    if(FSM!=fsm_moveto):
                        FSM = fsm_moveto
                        if(JobREGIM==DEBUG): print "load fsm_moveto"
                        FSM.reset()  
                        
                if(len(newline)>0): 
                    AppendRoute(newline)
                    Road = RouteProc(Route)
                    if(JobREGIM==DEBUG): 
                        print "\nLine: ", newline
                        print "Road: ", Road

            # **********************************************  2  **********************************************
            if(a.agent.id == 2 and regim==REPEAT):   # фильтрация данных локатора с переводом в ЛП
                pointlist = cLocator(a.agent.SuperLocator, 1)
                if(JobREGIM==DEBUG): print GlobalTimer, " Локатор 2: id = ", a.agent.id,  ", ", pointlist
                
                if (CurrPoint < 0):   
                    ObjFound = False
                    CurrPoint = 0       # начало пути
                    #tmu.SetAgentPos(1, 10, 10, 0)
                    tmu.KillAgent(1)
                    Way = WayProc(Route)  # преобразование пути к виду, понятному другому объекту (к автоматному виду)
                    
                    #tmu.SetAgentPos(2, XBegin, YBegin, 90)
                
                # Рефлекс
                if(sGetFL(a)<5 or sGetFR(a)<5):
                    if(JobREGIM==DEBUG): print "Reflex"
                    if(FSM!=fsm_reflex):
                        FSM = fsm_reflex
                        FSM.reset()    
                        
                if (FSM!=fsm_reflex or res==FSM_FINISHED):
                    name1, col1 = NextAuto(CurrPoint)    # определяет следующий автомат и цвет ориентира, который нужно обходить
                    name2, col2 = NextAuto(CurrPoint+1)  # определяет еще один ориентир (второй)
                    ObjX = FindObj(col1, pointlist)      # ищет ориентир указанного цвета среди видимых ориентиров
                    ObjY = FindObj(col2, pointlist)      # ищет следующий ориентир указанного цвета среди видимых ориентиров
                    if(JobREGIM==DEBUG): 
                        print "pointlist = ", pointlist, " name1 = ", name1, " col1 = ", col1
                        print "name2 = ", name2, "col2 = ", col2
                        print GlobalTimer, " NextAuto 1 = ", name1, "CurrPoint = ", CurrPoint, " ObjX = ", ObjX, " ObjY = ", ObjY

                    if (col1 != ObjColor):
                        if (ObjY[0]>0):  # если видит второй ориентир, то к первому можно не ходить 
                            CurrPoint +=1
                            name1, col1 = name2, col2
                            ObjX = ObjY
                    
                    if(ObjX[0]>0):  # вижу ориентир нужного цвета по направлению ObjX[0]
                        if(JobREGIM==DEBUG): print GlobalTimer, " вижу ориентир нужного цвета"
                        FSM = setFSM(name1)  # выбрали следующий автомат или тот же самый
                        if (prevAuto <> FSM or prevObj <> col1): 
                            prevAuto = FSM 
                            prevObj = col1
                            FSM.reset()  # reset, если выбрали другой автомат
                    else: 
                        if(JobREGIM==DEBUG): print GlobalTimer, "Не вижу нужного ориентира цвета ", col1
                        nextpoint = FindNextStep(CurrPoint, pointlist) # пытаемся найти следующую точку в пути среди тех, что видим
                        if(JobREGIM==DEBUG): print "nextpoint = ", nextpoint
                        if(nextpoint>0): 
                            CurrPoint = nextpoint
                            if(JobREGIM==DEBUG): print "CurrPoint = ", CurrPoint
                            name1, col1 = NextAuto(CurrPoint)    # определяет следующий автомат и цвет ориентира, который нужно обходить
                            ObjX = FindObj(col1, pointlist)      # ищет ориентир указанного цвета среди видимых ориентиров
                            FSM = setFSM(name1)  # выбрали следующий автомат
                            if(JobREGIM==DEBUG): print " следующий автомат = ", name1
                            FSM.reset()  # reset, если выбрали другой автомат
                        else: 
                            prevAuto = FSM 
                            turn = DirNextPoint(CurrPoint, ObjX)
                            if (turn == LEFT): FSM = fsm_aroundl
                            else: FSM = fsm_aroundr
                            if (prevAuto <> FSM): 
                                FSM.reset()  # reset, если выбрали другой автомат

                res = FSM.step() # автоматически считает такты - self.T++
                    
                if(JobREGIM==DEBUG):
                    print GlobalTimer, " step, ObjX = ", ObjX
                    print "### res, CurrPoint, ObjFound", res, CurrPoint, ObjFound
                if (CurrPoint>=len(Way) or FSM == fsm_endofpath or ObjFound):
		    eoj = True
		    break
      
                # Конец логики
                #-----------------------------

            GlobalTimer += 1

            r.sleep()

    print "LenRoute: ", len(Route)
    print "lenWay: ", len(Way)
    print "Route: ", Route
    print "Way: ", Way
    #print "количество пройденных точек"
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
    """
    Route = [[1, 2, 4, 4], [1, 3, 3, 5], [2, 3, 4, 4], [2, 4, 3, 5], [3, 3, 3, 4], [3, 4, 3, 5], [4, 2, 3, 4], [4, 4, 3, 5], 
	 [5, 2, 3, 4], [5, 3, 3, 5], [6, 2, 3, 4], [6, 4, 2, 5], [7, 1, 3, 4], [7, 4, 1, 6], [8, 4, 5, 6], [9, 4, 4, 6], [9, 5, 5, 7]]
    Way = RouteProc(Route)  # преобразование пути к виду, понятному другому объекту (к автоматному виду)
    for i in Route:
        print i
    print "Way: ", Way
    sys.exit(0)
    """
    # sic mainloop()
