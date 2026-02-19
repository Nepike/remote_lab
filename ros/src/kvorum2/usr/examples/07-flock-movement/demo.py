#!/usr/bin/env python3
# coding: utf-8
"""
  flock movement
  Простое стайное движение с локальным или глобальным лидером
  -- r.GetSrc()

  06.02.15, 17.05.2015, 22.01.2024
  Version 1.2
  LP 04.10.2024
"""
import os, sys, time
import roslib, rospy, time, random

from kvorum2 import gdic, tmurobot as tmu
from kvorum2.env import TEnv
from kvorum2.agent import TAgent, TSensor

###################################################################

Title = "Demo 1.04"

GlobalTimer = 0
Env = None
Agents = []

# Инициалиизация системы
def InitSystem(nodename, envfilename, mapfilename, agentfilename):
    global Agents

    # Инициалиизация ROS
    rospy.init_node(nodename)

    # Подписываемся на нужные топики
    tmu.InitUsersTopics(len(Agents))

    # Создаем роботов
    tmu.Robots = [ tmu.TRobot(a) for a in Agents ]

########################################################################
#
########################################################################

def Analyze(A):
    sit = {'fwd':0, 'back':0, 'left':0, 'right':0}
    f0 = A.GetSensor("TSOP_FWD")
    f1 = A.GetSensor("TSOP_BACK")
    f2 = A.GetSensor("TSOP_LEFT")
    f3 = A.GetSensor("TSOP_RIGHT")

    maxcode = max(f0, f1, f2, f3)
    if maxcode==f0: maxdir = 'fwd'
    if maxcode==f1: maxdir = 'back'
    if maxcode==f2: maxdir = 'left'
    if maxcode==f3: maxdir = 'right'

    sit['fwd'] = f0
    sit['back'] = f1
    sit['left'] = f2
    sit['right'] = f3

    return sit, maxcode, maxdir

########################################################################
#
########################################################################

def MakeReflex(a): 
    rdist = 5
    act, arg = tmu.PROC_GOFWD, 0
    refl = True
    if(sGetFL(a)<rdist and sGetFR(a)<rdist):
        act, arg = tmu.PROC_GOLEFT, 3 + random.randint(-1, 1)
    elif(sGetFL(a)<rdist):
        act, arg = tmu.PROC_GORIGHT,3 + random.randint(-1, 1)
    elif(sGetFR(a)<rdist):
        act, arg = tmu.PROC_GOLEFT, 3 + random.randint(-1, 1)
    else:
        refl = False
    return refl, act, arg

################################################################################
# MAIN
################################################################################

behavnum = 2

# Я сильнее или ничего не видно
# Тактика №1: ничего не делаю, стою на месте, только кручусь
# Тактика №2: куда-то иду

def main(envfilename, mapfilename, agentfilename):

    global Env, Agents, GlobalTimer

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

    while not rospy.is_shutdown():
        for r in tmu.Robots:
            # Если робот не дождался ответа на запрос сенсоров, то пропускаем ход
            if r.waits_for_sensors: continue

            r.RequestSensors()

            act, arg = tmu.PROC_GOFWD, 0

            refl, ract, rarg = MakeReflex(r)
            angle = 10
            sit, maxcode, maxdir = Analyze(r)
            mycode =  r.GetSrc(gdic.LEVEL_IR)
            isleader =  mycode>maxcode
            print(f"w{r.agent.id}: code={mycode}, maxcode={maxcode} mdir={maxdir} leader={isleader}")
            if isleader:
                #################################################################
                #
                # Лидер (я сильнее или ничего не видно)
                # Он просто гуляет сам по себе
                #
                #################################################################
                # Почти случайное блуждание
                act, arg = tmu.PROC_GOFWD, 0
                if (GlobalTimer%100 == 0):
                    act, arg = tmu.PROC_GOLEFT, angle*random.randint(-1, 1)
                else:
                    act, arg = tmu.PROC_GOFWD, 0
                # Устанавливаем состояние
                r.Make(tmu.PROC_SET_STATE, gdic.STAT_LEADER)
            else:
                #
                # Правило 1. Я слабее. Сближаемся с тем кто сильнее
                #
                if maxdir == 'fwd':
                    act, arg = tmu.PROC_GOFWD, 0
                elif maxdir == 'left':
                    act, arg = tmu.PROC_GOLEFT, angle + random.randint(-1, 1)
                elif maxdir == 'right':
                    act, arg = tmu.PROC_GORIGHT, angle + random.randint(-1, 1)
                elif maxdir == 'back':
                    act, arg = tmu.PROC_GOLEFT, angle + random.randint(-1, 1)
                # Устанавливаем состояние
                r.Make(tmu.PROC_SET_STATE, gdic.STAT_NORM)
            #
            # Правило 2. Стараемся держать дистанцию
            #
            normdist = 10
            if(sGetFL(r)<normdist or sGetFR(r)<normdist) and (act==tmu.PROC_GOFWD):
                act, arg = tmu.PROC_STOP, 0

            #################################################################
            #
            # Правило 3. Не сталкиваться. Рефлекс
            #
            #################################################################
            if refl: act, arg = ract, rarg
            r.Make(act, arg)

        GlobalTimer += 1
        if ((GlobalTimer % 100) == 0): print(GlobalTimer)

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
