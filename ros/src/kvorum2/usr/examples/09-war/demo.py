#!/usr/bin/env python3
# coding: utf-8
"""
  flock hunting war_demo
  -- FindIR
  
  477, 360
  06.02.15, 26.02.2017
  Version 1.05
  LP 04.10.2024
"""
import sys, random
import roslib, rospy

from kvorum2 import gdic, tmurobot as tmu
from kvorum2.env import TEnv
from kvorum2.agent import TAgent, TSensor

###################################################################

Title = "War Demo 1.05"

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

def FindFirst(R, code, delta, distlim):
    L = len(R.GetSensor("superlocator"))
    for n in range(L):
        dist, val = R.GetSensor("superlocator")[n]
        if(dist<=distlim or distlim<=0) and (val != 0):
            if((val>=code) and (val<code+delta)):
                return val
    return 0

def findat(R, a1, a2, code, delta, distlim):
    num = 0
    L = len(R.GetSensor("superlocator"))
    if(a1>=L or a2>L): return 0
    for n in range(a1, a2):
        dist, val = R.GetSensor("superlocator")[n]
        if(dist<=distlim or distlim<=0) and (val != 0):
            if((val>=code) and (val<code+delta)):
                num += 1
    return num

def Afind(id):
    global Agents
    for a in Agents:
        if(a.id == id): return a
    gdic.error("AFind: agent " + str(id) + " not found")
    return None
#
# q - номер квадранта:
#    0 - впереди
#    1 - сзади
#    2 - слева
#    3 - справа
#  углы a1, a2 - в диапазоне 0..360
#  [0]   180
#  [45]  135
#  [90]:  90
# [135]:  45
# [225]: -45
# [315]:-135
# [359]:-180
def FindIR(a, q, code, delta, distlim):
    val = 0
    if(q==0):
        a1, a2 = 45, 135
    if(q==1):
        a1, a2 = 225, 315
    if(q==2):
        a1, a2 = 135, 225
    if(q==3):
        a1, a2 = 0, 45
        a11, a21 = 315, 360
    val = findat(a, a1, a2, code, delta, distlim)
    if(val!=0): return val
    if(q==3):
        val = findat(a, a11, a21, code, delta, distlim)
    return val

########################################################################
#
########################################################################
def Analyze(A, friendcode, enemycode, num_frend, num_enemy, distlim):
    delta_frend = num_frend;
    delta_enemy = num_enemy;

    sit = {'fwd':[0,0], 'back':[0,0], 'left':[0,0], 'right':[0,0]}  

    sit['fwd'][0] =   FindIR(A, 0, friendcode, delta_frend, distlim)
    sit['back'][0] =  FindIR(A, 1, friendcode, delta_frend, distlim)
    sit['left'][0] =  FindIR(A, 2, friendcode, delta_frend, distlim)
    sit['right'][0] = FindIR(A, 3, friendcode, delta_frend, distlim)

    sit['fwd'][1] =   FindIR(A, 0, enemycode, delta_enemy, distlim)
    sit['back'][1] =  FindIR(A, 1, enemycode, delta_enemy, distlim)
    sit['left'][1] =  FindIR(A, 2, enemycode, delta_enemy, distlim)
    sit['right'][1] = FindIR(A, 3, enemycode, delta_enemy, distlim)

    nfriend = sit['fwd'][0]+sit['back'][0]+sit['left'][0]+sit['right'][0]
    nenemy  = sit['fwd'][1]+sit['back'][1]+sit['left'][1]+sit['right'][1]

    return sit, nfriend, nenemy

########################################################################
#
########################################################################

def MakeReflex(a): 
    rdist = 5
    act, arg = tmu.PROC_GOFWD, 0
    refl = True
    if(sGetFL(a)<rdist and sGetFR(a)<rdist):
        act, arg = tmu.PROC_GOLEFT, 5 + random.randint(-1, 1)
    elif(sGetFL(a)<rdist):
        act, arg = tmu.PROC_GORIGHT,5 + random.randint(-1, 1)
    elif(sGetFR(a)<rdist):
        act, arg = tmu.PROC_GOLEFT, 5 + random.randint(-1, 1)
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

    global TIP_T1, TIP_T2, IR_CODE_BASE_T1, IR_CODE_BASE_T2, NUM_T1, NUM_T2
    global SL_TSOP_FAR_DIST, SL_TSOP_NEAR_DIST

    # Инициалиизация системы
    print("Init environment... ")
    Env = TEnv(envfilename)

    print("Init map", mapfilename, "...")
    exec (open(mapfilename).read(), globals())

    print("Init agents", agentfilename, "...")
    exec (open(agentfilename).read(), globals())

    InitSystem('war_demo', envfilename, mapfilename, agentfilename)

    print("Start main loop")
    rate = rospy.Rate(Env.Rate)

    #####################################################
    # Основной цикл
    #####################################################
    DeadList = []
    while not rospy.is_shutdown():
        for r in tmu.Robots:
            if(r.agent.alive==False): continue
            # Если робот не дождался ответа на запрос сенсоров, то пропускаем ход
            if r.waits_for_sensors: continue

            my_id = r.agent.id
            r.RequestSensors()

            act, arg = tmu.PROC_GOFWD, 0
            refl, ract, rarg = MakeReflex(r)

            my_team = r.agent.shape

            if(my_team==TIP_T1):
                num_agent_frend = NUM_T1     # Количество друзей
                num_agent_enemy = NUM_T2     # Количество врагов
                friendcode = IR_CODE_BASE_T1
                enemycode = IR_CODE_BASE_T2
            else:
                num_agent_frend = NUM_T2
                num_agent_enemy = NUM_T1
                friendcode = IR_CODE_BASE_T2
                enemycode = IR_CODE_BASE_T1

            # Убираем дубли из суперлокатора
            SuperLocator = r.GetSensor("superlocator")
            L = len(SuperLocator)
            for n in range(L):
                dist, val = SuperLocator[n]
                if(val!=0):
                    for i in range(n+1, L):
                        dist2, val2 = SuperLocator[i]
                        if(val==val2):
                            SuperLocator[i] = (0, 0)
            r.SetSensorVal("superlocator", SuperLocator)


            print(f"{my_team[0]}.{r.agent.id}:\t", end='')

            #
            # Анализ ближайшего окружения
            #
            sit, nfriend, nenemy = Analyze(r, friendcode, enemycode, num_agent_frend, num_agent_enemy, SL_TSOP_NEAR_DIST)
            print(f"Tact:  friends={nfriend} enemies={nenemy} => ", end='')
            was_attack = False
            if(nenemy>0):
                was_attack = True
                refl = False # Отключаем рефлексы
                print("attack ", end='')
                # Пробуем напасть
                # Определяем id агента по IR-коду
                irc = FindFirst(r, enemycode, num_agent_enemy, SL_TSOP_NEAR_DIST)
                if(my_team==TIP_T1):
                    enemy_id = irc - IR_CODE_BASE_T2  + 1 + NUM_T1
                else:
                    enemy_id = irc - IR_CODE_BASE_T1  + 1

                print(f"enemy_id = {enemy_id} (irc={irc} enemycode={enemycode}) ", end='')

                # Определяем, кто сильнее
                aen = Afind(enemy_id)
                if aen.alive:
                    print(f"my oppenent is {enemy_id} ")
                    enemy_weight = aen.id
                    my_weight = r.agent.id

                    rn = random.randint(1,100)
                    mw = int(100.0*(nfriend + my_weight) / (nfriend + nenemy + my_weight + enemy_weight))
                    my_win = (rn<mw)
                    if my_win:
                        victim_id = enemy_id
                    else:
                        victim_id = my_id

                    print(f"*** Battle: {my_id} Vs {enemy_id} => kill {victim_id} ", end='')
                    tmu.KillAgent(victim_id)
                    DeadList.append(victim_id)
                else:
                    print(f"{enemy_id} is zombie ")
            else: # Никого нет
                print("...", end='')
                act, arg = tmu.PROC_GOFWD, 0
            print()

            #
            # Анализ стратегической ситуации
            #
            sit, nfriend, nenemy = Analyze(r, friendcode, enemycode, num_agent_frend, num_agent_enemy, SL_TSOP_FAR_DIST)
            print(f"\tStrat: friends={nfriend} enemies={nenemy} => ", end='')

            if not was_attack:
                angle = 10
                if(nenemy>0): # Бежим на врага
                    print("move to enemy ", end='')
                    if sit['fwd'][1]>0: # Враг впереди
                        act, arg = tmu.PROC_GOFWD, 0
                    elif sit['left'][1]>0: # Враг  слева
                        act, arg = tmu.PROC_GOLEFT, angle + random.randint(-1, 1)
                    elif sit['right'][1]>0: # Враг  справа
                        act, arg = tmu.PROC_GORIGHT, angle + random.randint(-1, 1)
                    elif sit['back'][1]>0: # Враг  сзади
                        act, arg = tmu.PROC_GOLEFT, angle + random.randint(-1, 1)
                elif(nfriend>0): # Сближаемся со своими
                    print("cohesion ", end='')
                    if sit['fwd'][0]>0: 
                        act, arg = tmu.PROC_GOFWD, 0
                    elif sit['left'][0]>0:
                        act, arg = tmu.PROC_GOLEFT, angle + random.randint(-1, 1)
                    elif sit['right'][0]>0:
                        act, arg = tmu.PROC_GORIGHT, angle + random.randint(-1, 1)
                    elif sit['back'][0]>0:  
                        act, arg = tmu.PROC_GOLEFT, angle + random.randint(-1, 1)
                else: # Никого нет
                    print("...", end='')
                    act, arg = tmu.PROC_GOFWD, 0
                    if GlobalTimer % 100 == 0:
                        act, arg = tmu.PROC_GOLEFT, random.randint(-2, 2)
            print()

            #################################################################
            # Рефлекс, возможные вненшие факторы
            if refl: act, arg = ract, rarg
            r.Make(act, arg)

        GlobalTimer += 1
        cnt = n1 = n2 = 0
        for r in tmu.Robots:
            if(not r.agent.alive): continue
            if(r.agent.id<=NUM_T1): n1 += 1
            else: n2 += 1
            cnt += 1
        print(f"{GlobalTimer}::: Survivors {cnt} from {len(tmu.Robots)}. Team1: {n1}, Team2: {n2}, DeadList: {DeadList}")

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
