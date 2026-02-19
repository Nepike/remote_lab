#!/usr/bin/env python3
# coding: utf-8
"""
  demo.py

  Demonstrates how to use an automatically generated fsm library.

  @author Rovbo Maxim
  @date 22.11.18
  LP 04.10.2024
"""
import os, sys, time
import roslib, rospy, time, random

from kvorum2 import gdic, tmurobot as tmu

from kvorum2.env import TEnv
from kvorum2.agent import TAgent, TSensor

from kvorum2.fsm_alternative import *
import fsm_fwd
import fsm_reflex

###################################################################

# Можно подгружать модуль именно таким образом, чтобы не было проблем с видимостью:
#  exec (open("src/kvorum/pylib/fsm.py").read())
# Но это не совсем красиво

###################################################################

Title = "ADemo 1.05"

GlobalTimer = 0
Env = None
Agents = []
CurrRobot = None

###################################################################
# Глобалы: текущий автомат, список автоматов
currFSM = None
FSMList = []

#
# Автоматные процедуры
#
def fProcGoFwd(): CurrRobot.Make(tmu.PROC_GOFWD, 0)

def fProcRandomTurn():
    if(currFSM.rand(2)==0):
        CurrRobot.Make(tmu.PROC_GOLEFT, 0)
    else:
        CurrRobot.Make(tmu.PROC_GORIGHT, 0)

def fProcGoBack(): CurrRobot.Make(tmu.PROC_STOP, 0)

def fProcGoStop(): CurrRobot.Make(tmu.PROC_NONE, 0)

def fProcGoRight(): CurrRobot.Make(tmu.PROC_GORIGHT, 0)

def fProcGoLeft(): CurrRobot.Make(tmu.PROC_GOLEFT, 0)

def aexit(): pass

#
# Автоматные предикаты условий перехода
#
def fIsLB(): return sGetFL(CurrRobot)<5

def fIsRB(): return sGetFR(CurrRobot)<5

#------------------------------------------------------------------------------
# Инициалиизация системы
#------------------------------------------------------------------------------
def InitSystem(nodename, envfilename, mapfilename, agentfilename):
    global Agents
    global FSMList

    # Инициалиизация ROS
    rospy.init_node(nodename)

    # Подписываемся на нужные топики
    tmu.InitUsersTopics(len(Agents))

    # Создаем роботов
    tmu.Robots = [ tmu.TRobot(a) for a in Agents ]

    #----------------------------------------------------------------------------
    # Создаем автоматы
    #----------------------------------------------------------------------------

    ##############################################################
    #
    # load fsm from library
    #
    ##############################################################
    f = fsm_reflex.FsmReflex()

    # connect fsm methods with robot functions
    f.GoBack = fProcGoBack
    f.Stop = fProcGoStop
    f.GoLeft = fProcGoLeft
    f.GoRight = fProcGoRight
    f.fIsLB = fIsLB
    f.fIsRB = fIsRB

    # optional tracing parameter (to see path in GUI)
    f.Trace = True

    # add fsm to the list
    FSMList.append(f)

    ##############################################################
    #
    # load fsm from library
    #
    ##############################################################
    f = fsm_fwd.FsmGoFwd()

    # connect fsm methods with robot functions
    f.goFwd = fProcGoFwd
    f.goStop = fProcGoStop
    f.aexit = aexit

    # optional tracing parameter (to see path in GUI)
    f.Trace = False

    # add fsm to the list
    FSMList.append(f)

def FindFSM(name):
    global FSMList
    for f in FSMList:
        if f.name==name:
            return f
    gdic.error("FindFSM: "+name+" not found")

################################################################################
# MAIN
################################################################################

def main(envfilename, mapfilename, agentfilename):

    global Env, GlobalTimer, CurrRobot
    global currFSM

    # Инициалиизация системы
    print("Init environment... ")
    Env = TEnv(envfilename)

    print("Init map", mapfilename, "...")
    exec (open(mapfilename).read(), globals())

    print("Init agents", agentfilename, "...")
    exec (open(agentfilename).read(), globals())

    InitSystem('demo', envfilename, mapfilename, agentfilename)

    print("Start main loop")
    rate = rospy.Rate(Env.Rate) # 50hz

    #####################################################
    # Основной цикл
    #####################################################

    fsm_reflex = FindFSM("Reflex")
    fsm_walk = FindFSM("GoFwd")
    currFSM = fsm_walk
    while not rospy.is_shutdown():
        for r in tmu.Robots:
            # Если робот не дождался ответа на запрос сенсоров, то пропускаем ход
            if r.waits_for_sensors: continue

            r.RequestSensors()
            #r.ShowStatus()
            CurrRobot = r

            # Рефлекс
            if(sGetFL(r)<5 or sGetFR(r)<5):
                if(currFSM != fsm_reflex):
                    print(f"{GlobalTimer} Reflex")
                    currFSM = fsm_reflex
                    currFSM.reset()

            res = currFSM.step()

            if res == FSM_FINISHED:
                print(f"{GlobalTimer} Walk")
                currFSM = fsm_walk
                currFSM.reset()

        GlobalTimer += 1

        rate.sleep()

    gdic.terminate_program()

################################################################################
#
################################################################################
if __name__ == '__main__':

    if (len(sys.argv) < 4):
        print("\n"+ Title, "\n\nUsage is:", sys.argv[0], "envfile mapfile agentfile")
        sys.exit(1)

    envfile = sys.argv[1]
    mapfile = sys.argv[2]
    agentfile = sys.argv[3]

    main(envfile, mapfile, agentfile)
