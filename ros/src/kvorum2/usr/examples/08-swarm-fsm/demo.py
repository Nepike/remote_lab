#!/usr/bin/env python3
# coding: utf-8
"""

  Рой роботов, управляемых автоматами
  (swarm+automaton(FSM))
  Роботы оснащены суперлокаторами

  06.02.2015, 13.01.2018
  Version 1.06
  LP 04.10.2024

"""
import os, sys, time
import roslib, rospy, time, random

from kvorum2 import gdic, tmurobot as tmu
from kvorum2 import fsm
from kvorum2.env import TEnv
from kvorum2.agent import TAgent, TSensor

from kvorum2.behavior import *

###################################################################

Title = "fht 1.05"

GlobalTimer = 0
Env = None
Agents = []

###################################################################

#
# Автоматные процедуры
#
def fProcGoFwd():
    CurrRobot.Make(tmu.PROC_GOFWD, 0)
    CurrRobot.curr_fsm.ResetT()

def fProcRandomTurn():
    if(CurrRobot.curr_fsm.rand(2)==0):
        CurrRobot.Make(tmu.PROC_GOLEFT, 0)
    else:
        CurrRobot.Make(tmu.PROC_GORIGHT, 0)
    CurrRobot.curr_fsm.ResetT()

def fProcGoBack():
    CurrRobot.Make(tmu.PROC_STOP, 0)
    CurrRobot.curr_fsm.ResetT()

def fProcStop():
    CurrRobot.Make(tmu.PROC_NONE, 0)
    CurrRobot.curr_fsm.ResetT()

def fProcGoRight():
    CurrRobot.Make(tmu.PROC_GORIGHT, 0)
    CurrRobot.curr_fsm.ResetT()

def fProcGoLeft():
    CurrRobot.Make(tmu.PROC_GOLEFT, 0)
    CurrRobot.curr_fsm.ResetT()

def fProcEat():
    food = fCheckFood()
    if food > 0:
        CurrRobot.Make(tmu.PROC_EAT)
        print("--", CurrRobot.agent.id, "eat")
    CurrRobot.curr_fsm.ResetT()

########################################################################
#
# Автоматные предикаты условий перехода
#
########################################################################

# Вспомогательные функции для обращения к датчикам
def fIsLB():
    rdist = 5
    s = CurrRobot.GetSensor("RF_FWD_LEFT")
    if(s==0): s = 100
    return s<rdist

def fIsRB():
    rdist = 5
    s = CurrRobot.GetSensor("RF_FWD_RIGHT")
    if(s==0): s = 100
    return s<rdist

def fCheckFood():
    s = CurrRobot.GetSensor("food_detector")!=0
    return s

def fCheckShadow():
    s = CurrRobot.GetSensor("light")==0
    return s

########################################################################
#
# Инициалиизация системы
#
########################################################################

#
# Создание автоматов
#
def CreateAllFSM():
    fsmlist = []
    # Рефлекс
    f = CreateFSM("A_REFLEX")
    f.Trace = True
    f.fIsLB = fIsLB
    f.fIsRB = fIsLB

    f.fProcGoLeft = fProcGoLeft
    f.fProcGoRight = fProcGoRight
    f.fProcGoBack = fProcGoBack
    f.fProcStop = fProcStop
    fsmlist.append(f)

    # Свободное блуждание
    f = CreateFSM("A_WALK")
    f.fProcGoFwd = fProcGoFwd
    f.fProcRandomTurn = fProcRandomTurn
    fsmlist.append(f)

    # Поиск пищи
    f = CreateFSM("A_SEARCH_FOOD")
    f.fProcStop = fProcStop
    f.fProcGoFwd = fProcGoFwd
    f.fProcRandomTurn = fProcRandomTurn
    f.fCheckFood = fCheckFood
    fsmlist.append(f)

    # Поиск тени
    f = CreateFSM("A_SEARCH_SHADOW")
    f.fCheckShadow = fCheckShadow
    f.fProcStop = fProcStop
    f.fProcGoFwd = fProcGoFwd
    f.fProcRandomTurn = fProcRandomTurn
    fsmlist.append(f)

    # Поедание пищи
    f = CreateFSM("A_EAT")
    f.fProcStop = fProcStop
    f.fProcEat = fProcEat
    fsmlist.append(f)

    # Сон
    f = CreateFSM("A_SLEEP")
    f.fProcStop = fProcStop
    f.fProcRandomTurn = fProcRandomTurn
    fsmlist.append(f)

    fsmlist.append(f)

    return fsmlist

def InitSystem(nodename, envfilename, mapfilename, agentfilename):
    global Agents

    # Инициалиизация ROS
    rospy.init_node(nodename)

    # Подписываемся на нужные топики
    tmu.InitUsersTopics(len(Agents))

    # Создаем роботов
    for a in Agents:

        r = tmu.TRobot(a)
        r.FSMList = CreateAllFSM()

        r.fsm_reflex = r.FindFSM("A_REFLEX")
        r.fsm_walk = r.FindFSM("A_WALK")
        r.fsm_search_food = r.FindFSM("A_SEARCH_FOOD")
        r.fsm_search_shadow = r.FindFSM("A_SEARCH_SHADOW")
        r.fsm_eat = r.FindFSM("A_EAT")
        r.fsm_sleep = r.FindFSM("A_SLEEP")

        r.curr_fsm = r.fsm_search_shadow

        tmu.Robots.append(r)

########################################################################
#
########################################################################

def findat(R, a1, a2, code, delta):
    num, maxcode = 0, 0
    L = len(R.GetSensor("superlocator"))
    if(a1>=L or a2>=L): return 0, 0
    for n in range(a1, a2):
        dist, val = R.GetSensor("superlocator")[n]
        if(val != 0):
            if((code == -1) or (abs(code-val) <= delta)):
                num+=1
                if(val>maxcode): maxcode = val
    return num, maxcode
#
# q - номер квадранта:
#    0 - впереди
#    1 - сзади
#    2 - слева
#    1 - справа
#  углы a1, a2 - в диапазоне 0..360
#  [0]   180
#  [45]  135
#  [90]:  90
# [135]:  45
# [225]: -45
# [315]:-135
# [359]:-180
def FindIR(a, q, code, delta):
    val = 0
    if(q==0):
        a1, a2 = 135, 225
    if(q==1):
        a1, a2 = 0, 45
        a3, a4 = 315, 359
    if(q==2):
        a1, a2 = 45, 135
    if(q==3):
        a1, a2 = 225, 315
    val, maxcode = findat(a, a1, a2, code, delta)
    if(val!=0): return val, maxcode
    if(q==1):
        val, maxcode = findat(a, a3, a4, code, delta)
    return val, maxcode

def Analyze(A, friendcode, enemycode):
    delta = 20

    sit = {'fwd':[0,0], 'back':[0,0], 'left':[0,0], 'right':[0,0]}  

    sit['fwd'][0], f1 = FindIR(A, 0, friendcode, delta)
    sit['back'][0], f2 = FindIR(A, 1, friendcode, delta)
    sit['left'][0], f3 = FindIR(A, 2, friendcode, delta)
    sit['right'][0], f4 = FindIR(A, 3, friendcode, delta)

    sit['fwd'][1], e1 = FindIR(A, 0, enemycode, delta)
    sit['back'][1], e2 = FindIR(A, 1, enemycode, delta)
    sit['left'][1], e3 = FindIR(A, 2, enemycode, delta)
    sit['right'][1], e4 = FindIR(A, 3, enemycode, delta)

    nfriend = sit['fwd'][0]+sit['back'][0]+sit['left'][0]+sit['right'][0]
    nenemy  = sit['fwd'][1]+sit['back'][1]+sit['left'][1]+sit['right'][1]
    maxfriend = max(f1, f2, f3, f4)

    return sit, nfriend, nenemy, maxfriend

################################################################################
#
# MAIN
#
################################################################################

def main(envfilename, mapfilename, agentfilename):

    global Env, GlobalTimer, CurrRobot

    # Инициалиизация системы
    print("Init environment... ")
    Env = TEnv(envfilename)

    print("Init map", mapfilename, "...")
    exec (open(mapfilename).read(), globals())

    print("Init agents", agentfilename, "...")
    exec (open(agentfilename).read(), globals())

    InitSystem('demo', envfilename, mapfilename, agentfilename)

    print("Start main loop")
    rate = rospy.Rate(10) # 50hz

    #####################################################
    # Основной цикл
    #####################################################

    while not rospy.is_shutdown():
        for r in tmu.Robots:
            if not r.agent.alive: continue
            # Если робот не дождался ответа на запрос сенсоров, то пропускаем ход
            if r.waits_for_sensors: continue

            CurrRobot = r
            r.RequestSensors()

            # Рефлекс
            if(fIsLB() or fIsRB()):
                if(r.curr_fsm != r.fsm_reflex):
                    r.pred_fsm = r.curr_fsm # Запоминаем предыдущий автомат
                    r.curr_fsm = r.fsm_reflex
                    r.curr_fsm.reset()

            res = r.curr_fsm.step()

            if res==FSM_FINISHED:
                if(r.curr_fsm == r.fsm_reflex):
                    r.curr_fsm = r.pred_fsm
                else:
                    if(r.curr_fsm == r.fsm_search_food):
                        r.curr_fsm = r.fsm_eat
                    else:
                        if(r.curr_fsm == r.fsm_search_shadow):
                            r.curr_fsm = r.fsm_search_food
                        else:
                            r.curr_fsm = r.fsm_search_shadow
                    r.curr_fsm.reset()

        GlobalTimer += 1

        rate.sleep()

    gdic.terminate_program()

################################################################################
#
################################################################################
if __name__ == '__main__':

    if (len(sys.argv) < 4):
        print("\n", Title, "\n\nUsage is:", sys.argv[0], "envfile mapfile agentfile")
        sys.exit(1)

    envfile = sys.argv[1]
    mapfile = sys.argv[2]
    agentfile = sys.argv[3]

    main(envfile, mapfile, agentfile)
