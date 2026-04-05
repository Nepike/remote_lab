#!/usr/bin/env python3
# coding: utf-8
"""
  demo.py

  Простая демонстрационная программа
  -- r.SetCurrVSpeed()

  06.02.15, 17.05.2015, 22.01.2024
  Version 2.1
  LP 04.10.2024

"""

import sys, random
import roslib, rospy

from kvorum2 import gdic
from kvorum2 import tmurobot as tmu

from kvorum2.env import TEnv
from kvorum2.agent import TAgent, TSensor

###################################################################

Title = "Simple Movement Demo 1.05"

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
    for a in Agents:
        r = tmu.TRobot(a)
        tmu.Robots.append(r)

################################################################################
# MAIN
################################################################################

def main(envfilename, mapfilename, agentfilename):

    global Env, Agents

    # Инициалиизация системы
    print("Init environment... ")
    Env = TEnv(envfilename)

    print("Init map", mapfilename, "...")
    exec (open(mapfilename).read())

    print("Init agents", agentfilename, "...")
    exec (open(agentfilename).read(), globals())

    InitSystem('demo', envfilename, mapfilename, agentfilename)

    print("Start main loop")
    rate = rospy.Rate(Env.Rate)

    #####################################################
    # Основной цикл
    #####################################################
    GlobalTimer = 0

    # Времена для смены направления движения
    T0 = 10   # Время ожидания (просто стоим и ничего не делаем)
    T = 200   # Каждые T тактов меняем направление

    while not rospy.is_shutdown():
        print(f"{GlobalTimer} ", end='')
        # Цикл по всем агентам
        for r in tmu.Robots:
            # Если робот не дождался ответа на запрос сенсоров, то пропускаем ход
            if r.waits_for_sensors: continue

            # Чтение сенсорных данных
            r.RequestSensors()

            # Положение робота в модельной среде
            position = r.GetSensor('position')
            x, y, a = position[0], position[1], position[2]
            print(f"pos={position}  x={x},y={y} a={a} enc_left={r.GetSensor('encoder_left')} enc_right={r.GetSensor('encoder_right')}", end='')

            # Действия
            # ...
            # r.Stop()
            # r.GoFwd()
            # r.GoBack()
            # r.GoFastLeft(self, ang=0)
            # r.GoFastRight(self, ang=0)
            # r.GoLeft(self, ang=0)
            # r.GoRight(self, ang=0)

            vlin = 0.75
            vang = 5

            if r.id==1:
                if GlobalTimer in range(T0, T0+T):
                    print("-- fwd")
                    r.SetCurrVSpeed(vlin, 0) # Go fwd
                    #r.GoFwd()
                elif GlobalTimer in range(T0+T, T0+T*2):
                    print("-- back")
                    r.SetCurrVSpeed(-vlin, 0) # Go back
                    #r.GoBack()
                elif GlobalTimer in range(T0+T*2, T0+T*3):
                    print("-- left")
                    r.SetCurrVSpeed(vlin, vang) # Go left
                elif GlobalTimer in range(T0+T*3, T0+T*4):
                    print("-- right")
                    r.SetCurrVSpeed(vlin, -vang) # Go right
                elif GlobalTimer in range(T0+T*4, T0+T*5):
                    print("-- fast left")
                    r.SetCurrVSpeed(vlin/2, vang*2) # Go right
                elif GlobalTimer in range(T0+T*5, T0+T*6):
                    print("-- fast right")
                    r.SetCurrVSpeed(vlin/2, -vang*2) # Go right
                elif GlobalTimer in range(T0+T*6, T0+T*7):
                    print("-- stop")
                    r.SetCurrVSpeed(0, 0) # Stop
                else:
                    T0 = GlobalTimer

        GlobalTimer += 1

        rate.sleep()

    gdic.terminate_program()

################################################################################
#
################################################################################
if __name__ == '__main__':

    if (len(sys.argv) < 4):
        print("\n"+Title+"\n\nUsage is: "+sys.argv[0]+" envfile mapfile agentfile")
        sys.exit(1)

    envfile = sys.argv[1]
    mapfile = sys.argv[2]
    agentfile = sys.argv[3]

    main(envfile, mapfile, agentfile)
