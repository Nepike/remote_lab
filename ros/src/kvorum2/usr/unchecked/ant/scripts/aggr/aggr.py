#!/usr/bin/env python
# coding: utf-8
"""
  aggr.py

  22.02.17
  Version 1.01
  LP 22.06.2017

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

Title = "Aggr 1.05"

###################################################################

MAXSTEPS = 300  # количество шагов моделирования
RAZMER = 100    # Максимальное количество роботов в группе

# Задание начального значения random
random.seed(63)  

FoodColor1  = 1  # цвет еды!!!!!!!!!!!!!!!!!!!!!!!!!!
FoodColor2  = 2  # цвет еды!!!!!!!!!!!!!!!!!!!!!!!!!!
GlobalTimer = 0
Env = None
Agents = []
CurrAgent = None
CurrFSM = None

MyHome = [ 0, 0, 0 ] # дом как объект: направление, расстояние, цвет 

# Глобалы
DEBUG = "DEBUG"
WORK  = "WORK"    # режим работы: печатает только данные о путях

JobREGIM = WORK  # режим отладки: печатает все

# Расчет агрессивности
ONLYDIST = "ONLYDIST"
TIMEDIST = "TIMEDIST"
Arule = ONLYDIST

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

#_--------------------------------------------------------------------------------------------------------
# Автоматные процедуры
#
def fProcGoFwd():
    CurrAgent.Make(tmu.PROC_GOFWD, 0)

def fProcRandomTurn():
    if( CurrFSM.rand(2)==0):      #random.choice(0,1)==0):        
        CurrAgent.Make(tmu.PROC_GORIGHT, 5+random.randint(1,10))
    else:
        CurrAgent.Make(tmu.PROC_GOLEFT, 5+random.randint(1,10))

def fProcRandomTurn2():
    if( CurrFSM.rand(2)==0):      #random.choice(0,1)==0):        
        CurrAgent.Make(tmu.PROC_GORIGHT, 170+random.randint(1,10))
    else:
        CurrAgent.Make(tmu.PROC_GOLEFT, 170+random.randint(1,10))

def fProcGoBack():
    CurrAgent.Make(tmu.PROC_GOBACK, 0)

def fProcGoStop():
    CurrAgent.Make(tmu.PROC_STOP, 0)

def fProcGoRight():
    CurrAgent.Make(tmu.PROC_GORIGHT, 5+random.randint(1,10))

def fProcGoLeft():
    CurrAgent.Make(tmu.PROC_GOLEFT, 5+random.randint(1,10))
    
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
  
def fLessAggr():
    return CurrAgent.agent.less_aggr

def fHomeDist(X):
    global JobREGIM
    if(JobREGIM==DEBUG):     print "dist for object = ", X[1]
    return X[1]

def ProcComeBack():
    # вернулись домой
    CurrAgent.agent.status = S_WALKING
    alfa = CurrAgent.agent.pos[2]+180-random.randint(1,30)
    tmu.SetAgentPos(CurrAgent.agent.id, CurrAgent.agent.pos[0], CurrAgent.agent.pos[1], alfa)
    print CurrAgent.agent.id, "alfa", alfa

def SingleLessAggress(r, nb):
    a_my = r.alevel
    aid = r.id//RAZMER
    for i in range(0, len(nb)):
        b = AFind(nb[i])
        if not(b is None): 
            if aid == nb[i]//RAZMER and a_my < b.alevel:  # это агент из моей группы, моя агрессивность ниже
                # и он близко (5 и менее клеток по x и y
                if abs(r.pos[0] - b.pos[0]) < 6 and abs(r.pos[1] - b.pos[1]) < 6:
                    return True
    return False
  
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

def fProcTurnToHome():    
    # Определяем направление на муравейник и устанавливаем угол поворота на него
    alfa = Angle(CurrAgent)
    if CurrAgent.agent.pos[0] != alfa:
        tmu.SetAgentPos(CurrAgent.agent.id, CurrAgent.agent.pos[0], CurrAgent.agent.pos[1], alfa)

def CheckColor():
    global MyHome, CurrAgent
    return sGetColor(CurrAgent) != MyHome[2]
  
def CreateAutomation():
    fsmList = []
    # Создаем автоматы
    f = TAutomaton("Walk", ["S", "1", "2", "3"], ["T"], "S")  # почему состояние Т не входит в список состояний?
    f.Trace = False
    f.SetTCnt(2)
    f.addRule(TRule("S", "1", "True",          "fProcGoFwd(); self.ResetT()"))
    f.addRule(TRule("1", "1", "self.CheckT()", "fProcGoFwd()"))
    f.addRule(TRule("1", "2", "else",          "fProcRandomTurn(); self.ResetT()"))
    f.addRule(TRule("2", "2", "self.CheckT()", ""))
    f.addRule(TRule("2", "3", "else",          "fProcGoFwd(); self.ResetT()"))
    f.addRule(TRule("3", "3", "self.CheckT()", ""))
    f.addRule(TRule("3", "S", "else",          ""))    

    fsmList.append(f)

    f = TAutomaton("ComeBack", ["S", "A", "T"], ["T"], "S")
    f.SetTCnt(2)
    f.Trace = False
    f.addRule(TRule("S", "A", "True",             "fProcTurnToHome(); fProcGoFwd()"))
    f.addRule(TRule("A", "A", "CheckColor() ",    "fProcTurnToHome()"))
    f.addRule(TRule("A", "T", "else",             "ProcComeBack()"))

    fsmList.append(f)
    """
    f = TAutomaton("Obstacle", ["S", "1", "B", "L", "R", "T"], ["T"], "S")
    f.SetTCnt(2)
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
    """
    # обход препятствий: если препятствием является агент, то отходит тот, чья агрессивность ниже
    f = TAutomaton("Obstacle", ["S", "T"], ["T"], "S")
    f.SetTCnt(2)
    f.addRule(TRule("S", "T", "(fIsLB() or fIsRB()) and fLessAggr()", "fProcGoBack()"))
#    f.addRule(TRule("S", "T", "(fIsLB() or fIsRB()) and not fLessAggr()", "fProcGoFwd()"))
    f.addRule(TRule("S", "T", "else",                "fProcGoStop()"))
    """
    f.addRule(TRule("S", "T", "fIsLB() and fIsRB()", "fProcRandomTurn2()"))
    f.addRule(TRule("S", "S", "fIsLB()",             "fProcGoRight()"))
    f.addRule(TRule("S", "S", "fIsRB()",             "fProcGoLeft()"))
    f.addRule(TRule("S", "T", "else",                "fProcGoStop()"))
    """
    
    fsmList.append(f)

    return fsmList

#------------------------------------------------------------------------------
# Инициалиизация системы
def InitSystem(nodename, envfilename, mapfilename, agentfilename):
    global Agents, FSMList, MyHome

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

def FindFSM(r, name):   
    for f in r.FSMList:
        if f.Name==name:
            return f
    gdic.error("FSM "+name+" not found")

def LoadFSM(r, name):
    f = FindFSM(r, name)
    f.reset()
    return f

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
    for i in range(0, len(nb)):
        if aid//RAZMER <> nb[i]//RAZMER: 
            b = AFind(nb[i])
            if not(b is None): 
                if(JobREGIM==DEBUG): print "#####", aid, " -- ", nb[i], " -- ", al, " -- ", b.alevel
                if al < b.alevel: return True
    return False

def LessAggress(r, nb):
    a_my = r.agent.alevel
    a_notmy = 0
    aid = r.agent.id//RAZMER
    for i in range(0, len(nb)):
        b = AFind(nb[i])
        if not(b is None): 
            if aid == nb[i]//RAZMER: a_my += b.alevel
            else: a_notmy += b.alevel
    if a_notmy == 0: return False
    n = 100*a_my/(a_my+a_notmy)
    m = random.randint(1, 100)
    #print "a_my, a_notmy", a_my, a_notmy, "n, m", n, m
    if m > n: return True
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
    global CurrAgent, MyHome
    global JobREGIM
    global CurrFSM

    nnn = 0
    
    SD =   [ 0, 0, 0, 0, 0, 0, 0, 0, 0 ] # массив для хранения статистики
    stat = [ 0, 0, 0, 0, 0, 0, 0, 0, 0 ] # массив для хранения промежуточной статистики (за такт)
    Fgr =  [ 0, 0, 0, 0, 0 ] # расстояние до найденной еды
    NoGrp = 0 # номер группы
    
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
        NoGrp = a.agent.id//RAZMER   # номер группы
        a.agent.alevel = 0.0         # уровень агрессивности
        a.agent.alimit = 0.5         # порог срабатывания
        a.agent.atime  = 0           # время неагрессивного поведения
        a.agent.status = S_WALKING   # текущее состояние агента
        # агент запоминает свой дом - это та точка, в которой он появился впервые (x, y, color)
        a.agent.ahome = a.agent.pos
        a.agent.ahome[0] = a.agent.pos[0]//1  # координата дома (x)
        a.agent.ahome[1] = a.agent.pos[1]//1  # координата дома (y)
        a.agent.ahome[2] = NoGrp+10  # цвет дома
        a.agent.adist = 0            # сумма расстояний от дома
        a.agent.adamage = 0          # количество поражений
        a.agent.pp_my = 0            # количество приемов своей пищи
        a.agent.pp_notmy = 0           # количество приемов чужой пищи
        NUM_i[NoGrp] +=1               # считаем количество роботов в группе
        a.FSMList = CreateAutomation() # Текущий автомат, список автоматов
        a.FSM = LoadFSM(a, "Walk")
        a.agent.prevcolor = 0        # цвет фона на предыдущем шаге
        a.agent.less_aggr = False    # признак "моя агрессивность ниже, чем у того, кто мешает мне пройти"
        
    # Максимальное количество групп роботов - 4, минимальное - 2.
    # Если их меньше 4-х, NUM_i[3] и NUM_i[4] надо установить в 1, т.к. на них делят
    if NUM_i[3] == 0: NUM_i[3]=1
    if NUM_i[4] == 0: NUM_i[4]=1
    
    while not rospy.is_shutdown():
        if (GlobalTimer>MAXSTEPS or eoj): break
        for a in tmu.Robots:
            NoGrp = a.agent.id//RAZMER
            CurrAgent = a
            CurrFSM = a.FSM
            m = a.agent.id//RAZMER
            
            pointlist = []
            splist = []
            plcolor = 0
            
            a.RequestAllSensors(immediate = True, req_main_sensors=True, req_locator=True, req_super_locator=True, req_i2cdata=True, req_tsoprc5=False, req_registers=False)
            
            # Определить текущее местоположение
            ppp = len(a.agent.MainSensors)-1
            a.agent.pos = [ a.agent.MainSensors[ppp-2], a.agent.MainSensors[ppp-1], a.agent.MainSensors[ppp] ]
            
            plcolor = sGetColor(a)  # видит цвет, на котором стоит
            #print "-------------------------------", plcolor, a.agent.pos
            if (plcolor == 0): plcolor = a.agent.prevcolor
            if (plcolor != 0):
                #print nnn, GlobalTimer, a.agent.id, plcolor
                nnn +=1
                if (m == plcolor or m+10==plcolor): a.agent.pp_my += 1
                else: a.agent.pp_notmy += 1
                a.agent.prevcolor = plcolor

            pointlist = a.agent.Locator   # видит соседей
            # Определить расстояние до дома
            S = ADist(a.agent.ahome[0], a.agent.ahome[1], a.agent.pos[0], a.agent.pos[1])
            a.agent.adist += S # сумма расстояний от дома
            Fgr[m] += S   # суммировать расстояние от дома до местоположения по группам
                   
            MyHome = [ 0, S//1, a.agent.ahome[2] ] # дом как объект: направление, расстояние, цвет 

            # Определить новый уровень агрессивности
            if (Arule == ONLYDIST): 
                a.agent.alevel = min (1, 1.0/(S/5+1))  # только расстояние
            if (Arule == TIMEDIST): 
                a.agent.alevel = min (1, 1.0/(S/5+1)+a.agent.atime*0.01)  # расстояние и время

            # Посмотреть на соседей и сравнить уровни агрессивности
            #"""
            neighbour = AgentList(pointlist) 
            if(JobREGIM==DEBUG): print "neighbour: ", neighbour
            #if notLeader(a.agent.id, a.agent.alevel, neighbour):
            if SingleLessAggress(a.agent, neighbour):  # сравнение своей агрессивности с агрессивностью агента, стоящего рядом со мной
                a.agent.less_aggr = True         # моя агрессивность ниже
            else: a.agent.less_aggr = False
            if LessAggress(a, neighbour):
                if(JobREGIM==DEBUG): 
                    print "^^^^^^^^^", a.agent.id, " go to home ", a.agent.ahome
                tmu.SetAgentPos(a.agent.id, a.agent.ahome[0], a.agent.ahome[1], a.agent.pos[2]+random.randint(1, 15) )
                a.agent.atime = 0  # обнуляем время без агрессии
                a.agent.adamage += 1 # количество "поражений"
            #"""    
            if (JobREGIM==DEBUG): print "agent ",a.agent.id,", ",a.agent.alevel,", ", S 

            # Препятствие
            if(sGetFL(a)<5 or sGetFR(a)<5):
                if(a.FSM.Name != "Obstacle"):
                    if(JobREGIM==DEBUG): 
                        print "Obstacle", a.agent.id
                    a.FSM = LoadFSM(a, "Obstacle")
                             
            res = a.FSM.step() # автоматически считает такты - self.T++
                                
            if res==FSM_FINISHED:
                if(JobREGIM==DEBUG): 
                    print "load fsm_walk"
                a.FSM = LoadFSM(a, "Walk")
                
            a.agent.atime += 1  # увеличиваем время без агрессии
            if(JobREGIM==DEBUG): 
                print a.agent.id, " S = ", S, " pos =", a.agent.pos, " ahome=", a.agent.ahome
            stat[NoGrp] += S   # суммировать расстояние по группам

            if a.agent.id == 100: 
                #if(JobREGIM==DEBUG): 
                if(JobREGIM==DEBUG): print stat[1]//NUM_i[1], stat[2]//NUM_i[2], stat[3]//NUM_i[3], stat[4]//NUM_i[4]
                for i in range(0, len(stat)):
                    SD[i] += stat[i]  # суммировать расстояние по группам
                    stat[i] = 0
                   
            if a.agent.id == RAZMER: 
                GlobalTimer += 1
                #print "agent ",a.agent.id,", ",a.agent.alevel,", ", S 
            r.sleep()

    # вывод данных по группам (номер группы равен первой цифре идентификатора робота)
    #print "id Приемов своей пищи, Приемов чужой пищи , среднее расстояние, количество поражений"
    for i in range(1,4):
        my = 0
        nm = 0
        for a in tmu.Robots:
            if a.agent.id//RAZMER == i: 
                my += a.agent.pp_my
                nm += a.agent.pp_notmy
        print my, nm
        
       
    print "SD = ", SD[1]//MAXSTEPS, SD[2]//MAXSTEPS

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
