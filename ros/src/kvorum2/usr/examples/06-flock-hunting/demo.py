#!/usr/bin/env python3
# coding: utf-8
"""
  flock hunting

  Стайная охота с определением локального лидера.
  Роботы оснащены суперлокаторами.

  06.02.15, 17.05.2015
  Version 1.04
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

########################################################################
#
########################################################################

def MakeReflex(a):
    rdist = 5
    act, arg = tmu.PROC_GOFWD, 0
    refl = True
    if(sGetFL(a)<rdist and sGetFR(a)<rdist):
        act, arg = tmu.PROC_GOLEFT, 2 + random.randint(-1, 1)
    elif(sGetFL(a)<rdist):
        act, arg = tmu.PROC_GORIGHT,2 + random.randint(-1, 1)
    elif(sGetFR(a)<rdist):
        act, arg = tmu.PROC_GOLEFT, 2 + random.randint(-1, 1)
    else:
        refl = False
    return refl, act, arg

################################################################################
#
# MAIN
#
################################################################################

def main(envfilename, mapfilename, agentfilename):

    global Env, Agents, GlobalTimer
    global HUNTER_IR_CODE

    # Инициалиизация системы
    print("Init environment... ")
    Env = TEnv(envfilename)

    print("Init map", mapfilename, "...")
    exec (open(mapfilename).read(), globals())

    print("Init agents", agentfilename, "...")
    exec (open(agentfilename).read(), globals())

    InitSystem('demo', envfilename, mapfilename, agentfilename)

    print("Start main loop")
    rate = rospy.Rate(10)

    print(HUNTER_IR_CODE)
    #####################################################
    # Основной цикл
    #####################################################

    while not rospy.is_shutdown():
        for r in tmu.Robots:
            if not r.agent.alive: continue
            # Если робот не дождался ответа на запрос сенсоров, то пропускаем ход
            if r.waits_for_sensors: continue

            r.RequestSensors()

            #
            # Рефлексы
            #
            refl, ract, rarg = MakeReflex(r)
            #################################################################
            # Хищник
            #################################################################
            if(r.agent.shape==TIP_HUNTER):
                sit, nfriend, nenemy, maxfriend = Analyze(r, HUNTER_IR_CODE, VICTIM_IR_CODE)
                isLeader = (r.agent.GetSrc(gdic.LEVEL_IR)>maxfriend)
                print(f"h{r.agent.id}: friends={nfriend} enemies={nenemy} maxfriend={maxfriend} leader={isLeader}")
                angle = 10
                if(nenemy>0): # Ловим
                    if sit['fwd'][1]>0: # Жертва впереди
                        act, arg = tmu.PROC_GOFWD, 0
                    elif sit['left'][1]>0: # Жертва слева
                        act, arg = tmu.PROC_GOLEFT, angle + random.randint(-1, 1)
                    elif sit['right'][1]>0: # Жертва справа
                        act, arg = tmu.PROC_GORIGHT, angle + random.randint(-1, 1)
                    elif sit['back'][1]>0: # Жертва сзади
                        act, arg = tmu.PROC_GOLEFT, angle + random.randint(-1, 1)
                elif(nfriend>0): # Сближаемся со своими
                    # Агент слабее. Сближаемся
                    if(not isLeader):
                        if sit['fwd'][0]>0: act, arg = tmu.PROC_GOFWD, 0
                        elif sit['left'][0]>0:  act, arg = tmu.PROC_GOLEFT, angle + random.randint(-1, 1)
                        elif sit['right'][0]>0: act, arg = tmu.PROC_GORIGHT, angle + random.randint(-1, 1)
                        elif sit['back'][0]>0:  act, arg = tmu.PROC_GOLEFT, angle + random.randint(-1, 1)
                        r.agent.SetState(gdic.STAT_NORM)
                    else:
                    # Агент сильнее. Лидер не подчиняется правилам сближения
                        r.agent.SetState(gdic.STAT_LEADER)
                        act, arg = tmu.PROC_GOFWD, 0
                else: # Ничего нет
                    act, arg = tmu.PROC_GOFWD, 0
                    # Рефлексы
                    if refl:
                        act, arg = ract, rarg
                        print(f"h{r.agent.id}: reflex!")
                # Пробуем съесть
                nv = GetLightSensor(r)
                if(nv!=0):
                    print(f"!!! hunter {r.agent.id} found victim {nv}")
                    act, arg = tmu.PROC_KILL, nv

            #################################################################
            # Жертва
            #################################################################
            else:
                sit, nfriend, nenemy, maxfriend = Analyze(r, VICTIM_IR_CODE, HUNTER_IR_CODE)
                angle = 10
                if(nenemy>0): # Убегаем
                    if sit['fwd'][1]>0: # Опасность впереди
                        act, arg = tmu.PROC_GOBACK, 0 #tmu.PROC_GOLEFT, angle + random.randint(-1, 1)
                    elif sit['left'][1]>0: # Опасность слева
                        act, arg = tmu.PROC_GORIGHT, angle + random.randint(-1, 1)
                    elif sit['right'][1]>0: # Опасность справа
                        act, arg = tmu.PROC_GOLEFT, angle + random.randint(-1, 1)
                    elif sit['back'][1]>0: # Опасность сзади
                        act, arg = tmu.PROC_GOFWD, 0
                elif(nfriend>0): # Сближаемся со своими
                    if sit['fwd'][0]>0: act, arg = tmu.PROC_GOFWD, 0
                    elif sit['left'][0]>0:  act, arg = tmu.PROC_GOLEFT, angle + random.randint(-1, 1)
                    elif sit['right'][0]>0: act, arg = tmu.PROC_GORIGHT, angle + random.randint(-1, 1)
                    elif sit['back'][0]>0:  act, arg = tmu.PROC_GOLEFT, angle + random.randint(-1, 1)
                else: # Ничего нет
                    act, arg = tmu.PROC_GOFWD, 0

                # Рефлексы
                if refl:
                    act, arg = ract, rarg
                    print(f"v{r.agent.id}: reflex!")
            #################################################################
            r.Make(act, arg)

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
