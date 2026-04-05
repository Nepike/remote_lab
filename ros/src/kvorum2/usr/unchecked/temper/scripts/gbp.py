#!/usr/bin/env python
# coding: utf-8
"""
  t2.py

  Эмоционально-темпераментный робот
  Использует управляющие автоматы
  06.02.15/04.09.2016
  Version 1.08
  LP 27.04.2018

"""
import os, sys, time
import roslib, rospy, random
import numpy

import gdic, ctb
from env import TEnv
from agent import TAgent, TSensor
import tmurobot as tmu

from fsm import *
import behavior

import ekr16 as emolib

######################################################################

Title = "T2 Emo+Temper 1.08"

GlobalTimer = 0

Env = None
Agents = []

pub_cmd = None

#
# Компоненты эмоциональной СУ
#

Params = emolib.TTEParams()
Emo = emolib.TEmoKernel(Params)

Needs = emolib.TNeeds()
Sensors = emolib.TSens()

#
# Коэффициенты возбуждения и затухания
#
class TCoeffExtDegr:
    def __init__(self):
        # Возбуждение (чем больше коэффициент, тем быстрее возбуждается)
        self.k_ext_food      = 0.75   # 
        self.k_ext_danger    = 0.50   # 
        self.k_ext_obstacle  = 0.50   # 
        self.k_ext_hungry    = 0.001

        # Затухание (чем больше коэффициент, тем быстрее затухает)
        self.k_degr_food     = 0.25  # Пища
        self.k_degr_danger   = 0.25  # Опасность
        self.k_degr_obstacle = 0.25  #
        self.k_degr_hungry   = 0.50

CED = TCoeffExtDegr()

######################################################################
#
# Вспомогательные функции для обращения к датчикам
#
######################################################################

def sGetObstLeft(A):
    s = A.agent.MainSensors[0]
    if(s==0): s = 100
    return s

def sGetObstRight(A):
    s = A.agent.MainSensors[1]
    if(s==0): s = 100
    return s

def sGetFood(A):
    s = A.agent.MainSensors[2]
    return s

def sGetFoodLeft(A):
    s = A.agent.MainSensors[3]
    return s

def sGetFoodRight(A):
    s = A.agent.MainSensors[4]
    return s

def sGetLightDangerLeft(A):
    s = A.agent.MainSensors[5]
    if(s==0): s = 100
    return s

def sGetLightDangerRight(A):
    s = A.agent.MainSensors[6]
    if(s==0): s = 100
    return s

def sGetShadowLeft(A):
    s = A.agent.MainSensors[5]
    if(s>0): s = 0
    else: s = 1
    return s

def sGetShadowRight(A):
    s = A.agent.MainSensors[6]
    if(s>0): s = 0
    else: s = 1
    return s

######################################################################
#
# Создание обобщенных автоматов
#
# Автоматные предикаты условий перехода и двигательные функции
#
######################################################################

# Пища
def fIsFood(): return sGetFood(behavior.CurrRobot)>0

def fDirFood():
    left = sGetFoodLeft(behavior.CurrRobot)>0
    right = sGetFoodRight(behavior.CurrRobot)>0
    if(left and right): return G_FWD
    if(left): return G_LEFT
    if(right): return G_RIGHT
    return G_NONE

######################################################################
# Рефлекс на препятствие
######################################################################
# Расстояния до препятствия
ReflexObstDist = 3   # Дистанция рефлекса
EscapeObstDist = 20  # Дистанция обнаружения препятствия (реакция убегания)
FoundObstDist = 10   # Дистанции "рядом с препятствием"

def fIsReflexObstLeft():
    return sGetObstLeft(behavior.CurrRobot)<ReflexObstDist

def fIsReflexObstRight():
    return sGetObstRight(behavior.CurrRobot)<ReflexObstDist

def fDirReflexObst():
    left = fIsReflexObstLeft()
    right = fIsReflexObstRight()
    if(left and right): return G_FWD
    if(left): return G_LEFT
    if(right): return G_RIGHT
    return G_NONE

######################################################################
# Избегание препятствия
######################################################################
def fIsEscapeObstLeft():
    return sGetObstLeft(behavior.CurrRobot)<EscapeObstDist

def fIsEscapeObstRight():
    return sGetObstRight(behavior.CurrRobot)<EscapeObstDist

def fDirEscapeObst():
    left = fIsEscapeObstLeft()
    right = fIsEscapeObstRight()
    if(left and right): return G_FWD
    if(left): return G_LEFT
    if(right): return G_RIGHT
    return G_NONE

######################################################################
# Движение к препятствию
######################################################################
def fObstIsNear():
    return (sGetObstLeft(behavior.CurrRobot)<FoundObstDist) or (sGetObstRight(behavior.CurrRobot)<FoundObstDist)

def fDirToObst():
    left = sGetObstLeft(behavior.CurrRobot)<100
    right = sGetObstLeft(behavior.CurrRobot)<100
    if(left and right): return G_FWD
    if(left): return G_LEFT
    if(right): return G_RIGHT
    return G_NONE

######################################################################
# Опасность (свет)
######################################################################

def fIsLightDangerLeft(): return sGetLightDangerLeft(behavior.CurrRobot)<100
def fIsLightDangerRight(): return sGetLightDangerRight(behavior.CurrRobot)<100

def fIsLightDanger():
    return (sGetLightDangerLeft(behavior.CurrRobot)<=1) or \
           (sGetLightDangerRight(behavior.CurrRobot)<=1)

def fDirLightDanger():
    left = fIsLightDangerLeft()
    right = fIsLightDangerRight()
    if(left and right): return G_FWD
    if(left): return G_LEFT
    if(right): return G_RIGHT
    return G_NONE

######################################################################
# Тень
######################################################################
def fIsShadow(): return (fIsShadowLeft() and fIsShadowRight())
def fIsShadowLeft(): return sGetShadowLeft(behavior.CurrRobot)>0
def fIsShadowRight(): return sGetShadowRight(behavior.CurrRobot)>0

def fDirShadow():
    left = fIsShadowLeft()
    right = fIsShadowRight()
    if(left and right): return G_FWD
    if(left): return G_LEFT
    if(right): return G_RIGHT
    return G_NONE

######################################################################
#
# Инициалиизация системы
#
######################################################################

#
# Создание автоматов
#
def CreateFSM():
    fsmlist = []
    #
    # Рефлекс на препятствие
    #
    f = behavior.CreateGBPEscape("Reflex")
    f.SetTCnt(10)
    f.Trace = False
    f.fsmDir = fDirReflexObst
    f.fsmMove = behavior.fEscapeGeneralProc

    fsmlist.append(f)

    #
    # Избегание препятствия (это не рефлекс)
    #
    f = behavior.CreateGBPEscape("EscapeObstacle")
    f.SetTCnt(10)
    f.Trace = False
    f.fsmDir = fDirEscapeObst
    f.fsmMove = behavior.fEscapeGeneralProc

    fsmlist.append(f)

    #
    # Свободное блуждание
    #
    f = TAutomaton("Walk", ["S", "1", "2", "3", "T"], ["T"], "S")
    f.Trace = False
    f.SetTCnt(10)
    f.GoFwd = behavior.fProcGoFwd
    f.RandomTurn = behavior.fProcRandomTurn
    f.addRule(TRule("S", "1", "True",          "self.GoFwd(); self.ResetT()"))
    f.addRule(TRule("1", "1", "self.CheckT()", ""))
    f.addRule(TRule("1", "2", "True",          "self.RandomTurn(); self.ResetT()"))
    f.addRule(TRule("2", "2", "self.CheckT()", ""))
    f.addRule(TRule("2", "3", "True",          "self.GoFwd(); self.ResetT()"))
    f.addRule(TRule("3", "3", "self.CheckT()", ""))
    f.addRule(TRule("3", "S", "True",          ""))

    fsmlist.append(f)

    #
    # Поедание пищи
    # Вырожденный автомат
    f = TAutomaton("Eat", ["S", "T"], ["T"], "S")
    f.Trace = False
    f.Eat = behavior.fProcEat
    f.addRule(TRule("S", "T", "True", "self.Eat()"))

    fsmlist.append(f)

    #
    # Сон
    #
    f = TAutomaton("Sleep", ["S", "1", "2", "3", "T"], ["T"], "S")
    f.Trace = False
    f.SetTCnt(10)
    f.Stop = behavior.fProcGoStop
    f.RandomTurn = behavior.fProcRandomTurn
    f.addRule(TRule("S", "1", "True",          "self.Stop(); self.ResetT()"))

    f.addRule(TRule("1", "1", "self.CheckT()", ""))
    f.addRule(TRule("1", "2", "True",          "self.RandomTurn(); self.ResetT()"))

    f.addRule(TRule("2", "2", "self.CheckT()", ""))
    f.addRule(TRule("2", "S", "True",          "self.ResetT()"))

    fsmlist.append(f)

    #
    # Поиск пищи
    #
    f = behavior.CreateGBPSearch("SearchFood")
    f.Trace = False
    f.fsmFound = fIsFood
    f.fsmDir = fDirFood
    f.fsmMove = behavior.fMoveToGeneralProc

    fsmlist.append(f)

    #
    # "Убегающий" от опасности (свет) автомат
    #
    f = behavior.CreateGBPEscape("EscapeLightDanger")
    f.Trace = False
    f.fsmDir = fDirLightDanger
    f.fsmMove = behavior.fEscapeGeneralProc

    fsmlist.append(f)

    #
    # Поиск тени
    #
    f = behavior.CreateGBPSearch("SearchShadow")
    f.Trace = False
    f.fsmFound = fIsShadow
    f.fsmDir = fDirShadow
    f.fsmMove = behavior.fMoveToGeneralProc

    fsmlist.append(f)

    #
    # Движение к препятствию
    #
    f = behavior.CreateGBPSearch("MoveToObst")
    f.Trace = False
    f.fsmFound = fObstIsNear
    f.fsmDir = fDirToObst
    f.fsmMove = behavior.fMoveToGeneralProc

    fsmlist.append(f)
    return fsmlist

def InitSystem(nodename, envfilename, mapfilename, agentfilename):
    global pub_cmd
    global Env, Agents

    print "Init environment... "
    Env = TEnv(envfilename)

    print "Init map", mapfilename, "..."
    exec (open(mapfilename).read())

    print "Init agents", agentfilename, "..."
    exec (open(agentfilename).read())

    # Инициалиизация ROS
    rospy.init_node(nodename)

    # Входной топик
    # Подписываемся на топик ardans, второй параметр - объем кэша отправки
    inpqlen = len(Agents)*7
    rospy.Subscriber("/ardans_topic", tmu.ans, tmu.rsans_callback, queue_size=
            max(1,inpqlen))

    # Выходной топик
    outqlen = len(Agents)*7
    pub_cmd = rospy.Publisher("/actions_topic", tmu.action, queue_size=outqlen)

    # Создаем роботов
    for a in Agents:
        r = tmu.TRobot(a,pub_cmd)
        r.FSMList = CreateFSM()
        r.pred_fsm = None
        r.pred_proc = tmu.PROC_NONE
        tmu.Robots.append(r)

######################################################################

def RobotShowStatus(r):
    print r.agent.id, "MainSensors", len(r.agent.MainSensors), r.agent.MainSensors
    return

#
# Показатели качества
#
CntEat = 0           # Счетчик еды
CntLightDanger = 0   # Счетчик опасности
CntHungry = 0        # Чувство голода

stepFood = 100
stepMove = 0.50
stepSleep = 0.25

def MakeAction(procnum):
    global CntHungry, CntEat, stepFood

    if (procnum != behavior.CurrRobot.pred_proc):
        behavior.CurrRobot.pred_proc = procnum
        # Загружаем автоматы
        if(procnum==emolib.PROC_SEARCH_FOOD):
            behavior.CurrRobot.LoadFSM("SearchFood")
        elif (procnum==emolib.PROC_EAT):
            behavior.CurrRobot.LoadFSM("Eat")
            # Имитируем поедание
            Sensors.hungry = ctb.edec(Sensors.hungry, CED.k_degr_hungry)
            CntHungry -= stepFood
            if(CntHungry<0): CntHungry = 0
            CntEat+=1
        elif(procnum==emolib.PROC_ESCAPE):
            behavior.CurrRobot.LoadFSM("EscapeObstacle")
        elif(procnum==emolib.PROC_SEARCH_SHADOW):
            behavior.CurrRobot.LoadFSM("SearchShadow")
        elif(procnum==emolib.PROC_SLEEP):
            behavior.CurrRobot.LoadFSM("Sleep")
        elif(procnum==emolib.PROC_WALK):
            behavior.CurrRobot.LoadFSM("Walk")
        elif(procnum==emolib.PROC_MOVE_TO_OBSTACLE):
            behavior.CurrRobot.LoadFSM("MoveToObst")
        elif(procnum==emolib.PROC_NONE):
            pass
        else:
            print "MakeAction: unknown procnum", procnum, emolib.ProcNames[procnum]
            gdic.pause("PEK")
            gdic.error("MakeAction: error")

    if(procnum==emolib.PROC_NONE): return

    # Рефлекс
    rfsm = behavior.CurrRobot.FindFSM("Reflex")
    if(fIsReflexObstLeft() or fIsReflexObstRight()):
        if(behavior.CurrRobot.curr_fsm != rfsm):
            # Сохраняем автомат
            print "============= REFLEX"
            behavior.CurrRobot.pred_fsm = behavior.CurrRobot.curr_fsm
            behavior.CurrRobot.curr_fsm = rfsm
            behavior.CurrRobot.curr_fsm.reset()

    res = behavior.CurrRobot.curr_fsm.step()

    if(res==FSM_FINISHED and behavior.CurrRobot.curr_fsm==rfsm):
        behavior.CurrRobot.curr_fsm = behavior.CurrRobot.pred_fsm
        print "============= RESTORE", behavior.CurrRobot.curr_fsm.Name

def ReadNeedsSensors():
    global CntHungry

    # Сенсор еды
    if(sGetFood(behavior.CurrRobot)>0):
        Sensors.food = ctb.einc(Sensors.food, CED.k_ext_food)

    # Сенсор препятствия
    maxdist = 20
    sL = sGetObstLeft(behavior.CurrRobot)
    sR = sGetObstRight(behavior.CurrRobot)
    if(sL>maxdist): resL = 0
    else: resL = float(maxdist-sL)/float(maxdist)
    if(sR>maxdist): resR = 0
    else: resR = float(maxdist-sR)/float(maxdist)
    ##&& k - коэффициент близости к препятствию. 0.4 - потому что иначе все время убегает от препятствия
    k = 0.25*max(resL, resR)
    Sensors.obstacle = ctb.einc(Sensors.obstacle, k)

    # Сенсор опасности
    maxdist = 20
    sL = sGetLightDangerLeft(behavior.CurrRobot)
    sR = sGetLightDangerRight(behavior.CurrRobot)
    if(sL>maxdist): resL = 0
    else: resL = float(maxdist-sL)/float(maxdist)
    if(sR>maxdist): resR = 0
    else: resR = float(maxdist-sR)/float(maxdist)
    k = 0.3*max(resL, resR)
    Sensors.danger = ctb.einc(Sensors.danger, k)

    # Сенсор голода
    Sensors.hungry = ctb.einc(Sensors.hungry, CED.k_ext_hungry)

    #
    # Деградация значений сенсоров (все, кроме голода)
    #
    Sensors.food = ctb.edec(Sensors.food, CED.k_degr_food)
    Sensors.danger = ctb.edec(Sensors.danger, CED.k_degr_danger)
    Sensors.obstacle = ctb.edec(Sensors.obstacle, CED.k_degr_obstacle)

######################################################################
# MAIN
######################################################################

def GetA(A, idx):
  R = []
  for i in range(0, len(A)):
      R.append(A[i][idx])
  return R

def GetStat(A, idx):
    X = GetA(A, idx)
    m = numpy.mean(X) # среднее значение
    v = numpy.var(X)  # дисперсия
    s = numpy.std(X)  # СКО
    return (m, v, s)

def S2Str(val):
    s = str(val[0]) + "\t" + str(val[1]) + "\t" + str(val[2]) + "\t"
    s.replace('.',',')
    return s

def S2Str2(name, val):
    s = name+":\t" + str(val[0]) + "\t" + str(val[1]) + "\t" + str(val[2]) + "\n"
    s.replace('.',',')
    return s

def main(envfile, mapfile, agentfile):
    global Env, GlobalTimer
    global CntHungry, CntLightDanger, CntEat, stepSleep, stepMove

    # Инициалиизация системы
    InitSystem('demo', envfile, mapfile, agentfile)

    print "Start main loop"
    r = rospy.Rate(10) # 50hz

    #####################################################
    # Основной цикл
    #####################################################

    outf = None
    if(not (Params.logfile is None)):
        outf = open(Params.logfile,"w")
        outf.write('T;CntEat;CntLightDanger;CntHungry;E;Proc;\n')
    RESULTS = []
    eoj = False
    while (not rospy.is_shutdown()) and (not eoj):

        print "***", GlobalTimer

        for robot in tmu.Robots:
            robot.RequestAllSensors(True)

            #print "----- Robot"
            #RobotShowStatus(robot)

            behavior.CurrRobot = robot

            ReadNeedsSensors()

            ###################################
            # Эмоциональное ядро
            ###################################
            currProcedure, EMOTION, nmax = Emo.Step(Sensors, Needs, Params)

            if(currProcedure is None): continue

            print "----- Kernel ----------------------------------"
            Emo.ShowStatus(Sensors, Needs)
            print "-----------------------------------------------"

            print '=> currProcedure: {:<8} ({}) nmax = {} ({}) E = {:5.2f}'.format(emolib.ProcNames[currProcedure], currProcedure, \
                nmax, emolib.ProcNames[nmax], EMOTION)

            ###################################
            #
            ###################################
            if(fIsLightDanger()): CntLightDanger+=1

            if(currProcedure==emolib.PROC_SLEEP):
                CntHungry += stepSleep
            else:
                CntHungry += stepMove

            CntHealth = 100      # Общее "здоровье"
            print "eat:", CntEat, "light:", CntLightDanger, "hungry:", CntHungry, "Health:", CntHealth

            if(not outf is None):
                s = str(GlobalTimer) + ';' + str(CntEat) + ';' + str(CntLightDanger) + ';'
                s = s + str(CntHungry) + ';' + str(CntHealth) + ';' + str(EMOTION)+ ';' + str(currProcedure)+'\n'
                # Заменяем '.' на ','
                s.replace('.',',')
                outf.write(s)

            RESULTS.append([CntEat, CntLightDanger, CntHungry, CntHealth, EMOTION, currProcedure])

            MakeAction(currProcedure)
            print "Curr FSM:", behavior.CurrRobot.curr_fsm.Name, behavior.CurrRobot.curr_fsm.q
            ###################################

        GlobalTimer += 1
        if(not (Params.timelim is None)):
            eoj = (GlobalTimer >= Params.timelim)

        r.sleep()

    if(not (outf is None)): outf.close()

    #
    # Вычисляем статистику и выводим
    #
    statCntEat = GetStat(RESULTS, 0)
    statCntLightDanger = GetStat(RESULTS, 1)
    statCntHungry = GetStat(RESULTS, 2)
    statCntHealth = GetStat(RESULTS, 3)
    statEMOTION = GetStat(RESULTS, 4)
    statcurrProcedure = GetStat(RESULTS, 5)

    f = None
    if(not (Params.logfile is None)):
        filename = Params.logfile+'_stat'
        f = open(filename,"w")
        f.write(str(Params))
        f.write("\n")
        f.write("Eat\t\t" + "LightDanger\t\t" + "Hungry\t\t" + "Health\t\t" + "EMOTION\t\t" + "Proc\t\t" + "\n")
        f.write(S2Str(statCntEat) + S2Str(statCntLightDanger) + S2Str(statCntHungry) + S2Str(statCntHealth)+\
                S2Str(statEMOTION)+S2Str(statcurrProcedure))
        f.write("\n")
        f.close()

    gdic.terminate_program()

######################################################################
#
#
#
######################################################################

if __name__ == '__main__':

    if (len(sys.argv) < 4):
        print "\n", Title, "\n\nUsage is:", sys.argv[0], "envfile mapfile agentfile [--log logfile] [--time tm]"
        sys.exit(1)

    envfile = sys.argv[1]
    mapfile = sys.argv[2]
    agentfile = sys.argv[3]


    # Разбираемся с параметрами
    for i in range(4, len(sys.argv)):
        if(sys.argv[i]=="--log"):
            Params.logfile = sys.argv[i+1]
        elif(sys.argv[i]=="--time"):
            Params.timelim = int(sys.argv[i+1])
        elif(sys.argv[i]=="--fb"):
            Params.coeff_fb = float(sys.argv[i+1])
        elif(sys.argv[i]=="--em"):
            Params.coeff_em = float(sys.argv[i+1])
        elif(sys.argv[i]=="--ex"):
            Params.excitation = float(sys.argv[i+1])
        elif(sys.argv[i]=="--inhibit"):
            Params.inhibit = int(sys.argv[i+1])
        elif(sys.argv[i]=="--need_food"):
            Needs.food = int(sys.argv[i+1])
        elif(sys.argv[i]=="--need_comfort"):
            Needs.comfort = int(sys.argv[i+1])
        elif(sys.argv[i]=="--need_save"):
            Needs.save = int(sys.argv[i+1])

    Params.show()

    main(envfile, mapfile, agentfile)
