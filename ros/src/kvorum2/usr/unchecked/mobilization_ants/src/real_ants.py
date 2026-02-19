#!/usr/bin/env python
# coding: utf-8
"""
  ants.py

  Ants mobilization in a foraging problem for Kvorum.

  @author Rovbo Maxim
  @version 1.00
  @date 15.04.2018
"""
import os, sys, time
import roslib, rospy, random
import numpy

import gdic
from env import TEnv
from agent import TAgent, TSensor, TEncoder
import tmurobot as tmu

from msg_kvorum.msg import senneed as TSenNeed

######################################################################

Title = "Ants Mobilization v1.0"

GlobalTimer = 0

Env = None
Agents = []

pub_cmd = None

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
    # "Убегающий" от опсаности (свет) автомат
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
        if(procnum==tmu.PROC_SEARCH_FOOD):
            behavior.CurrRobot.LoadFSM("SearchFood")
        elif (procnum==tmu.PROC_EAT):
            behavior.CurrRobot.LoadFSM("Eat")
            # Имитируем поедание
            CntHungry -= stepFood
            if(CntHungry<0): CntHungry = 0
            CntEat+=1
        elif(procnum==tmu.PROC_ESCAPE):
            behavior.CurrRobot.LoadFSM("EscapeObstacle")
        elif(procnum==tmu.PROC_SEARCH_SHADOW):
            behavior.CurrRobot.LoadFSM("SearchShadow")
        elif(procnum==tmu.PROC_SLEEP):
            behavior.CurrRobot.LoadFSM("Sleep")
        elif(procnum==tmu.PROC_WALK):
            behavior.CurrRobot.LoadFSM("Walk")
        elif(procnum==tmu.PROC_MOVE_TO_OBSTACLE):
            behavior.CurrRobot.LoadFSM("MoveToObst")
        elif(procnum==tmu.PROC_NONE):
            pass
        else:
            print "MakeAction: unknown procnum", procnum, tmu.ProcNames[procnum]
            gdic.pause("PEK")
            gdic.error("MakeAction: error")

    if(procnum==tmu.PROC_NONE): return

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
        behavior.CurrRobot.curr_fsm = pred_FSM
        print "============= RESTORE", behavior.CurrRobot.curr_fsm.Name

def ReadNeedsSensors():
    global CntHungry

    vsn = TSenNeed
    vsn.sen_excit = 1
    vsn.sen_inhibit = 1
    vsn.sen_reflex = 1

    vsn.need_food = 80 # 50
    vsn.need_comfort = 40
    vsn.need_save = 60

    # Сенсор еды
    if(sGetFood(behavior.CurrRobot)>0):
        vsn.sen_food = 100
    else:
        vsn.sen_food = 0

    # Сенсор препятствия
    maxdist = 20
    sL = sGetObstLeft(behavior.CurrRobot)
    sR = sGetObstRight(behavior.CurrRobot)
    if(sL>maxdist): resL = 0
    else:
      resL = int(100.0*float(maxdist-sL)/float(maxdist))
    if(sR>maxdist): resR = 0
    else:
      resR = int(100.0*float(maxdist-sR)/float(maxdist))
    vsn.sen_obstacle = max(resL, resR)

    # Сенсор опсаности
    maxdist = 20
    sL = sGetLightDangerLeft(behavior.CurrRobot)
    sR = sGetLightDangerRight(behavior.CurrRobot)
    if(sL>maxdist): resL = 0
    else:
      resL = int(100.0*float(maxdist-sL)/float(maxdist))
    if(sR>maxdist): resR = 0
    else:
      resR = int(100.0*float(maxdist-sR)/float(maxdist))
    vsn.sen_danger = max(resL, resR)

    # Сенсор голода
    if(CntHungry>100): CntHungry = 100
    vsn.sen_hungry = CntHungry

    return vsn

######################################################################
# MAIN
######################################################################
def main(envfile, mapfile, agentfile):
    global Env, GlobalTimer
    global stepSleep, stepMove

    # Инициалиизация системы
    InitSystem('Ant mobilization foraging', envfile, mapfile, agentfile)

    rate_hz = 10.
    print "Start main loop at {} Hz".format(rate_hz)
    r = rospy.Rate(rate_hz)

    #####################################################
    # Основной цикл
    #####################################################
    RESULTS = []
    eoj = False
    while (not rospy.is_shutdown()) and (not eoj):

        print "***", GlobalTimer

        for robot in tmu.Robots:
            robot.RequestAllSensors(True)

            #print "----- Robot"
            #RobotShowStatus(robot)

            #behavior.CurrRobot = robot

            #vsn = ReadNeedsSensors()

            #MakeAction(currProcedure)
            ###################################

        GlobalTimer += 1

        r.sleep()

    if(not (outf is None)): outf.close()

    gdic.terminate_program()

######################################################################
#
#
#
######################################################################

if __name__ == '__main__':

    if (len(sys.argv) < 3):
        print "\n", Title, "\n\nUsage is:", sys.argv[0], "envfile mapfile agentfile"
        sys.exit(1)

    envfile = sys.argv[1]
    mapfile = sys.argv[2]
    agentfile = sys.argv[3]

    main(envfile, mapfile, agentfile)
