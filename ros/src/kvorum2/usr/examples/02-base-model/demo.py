#!/usr/bin/env python3
# coding: utf-8
"""
  demo.py

  Простая демонстрационная программа
  -- SetSrc

  06.02.15, 17.05.2015
  Version 1.05
  LP 04.10.2024

"""

import os, sys, time
import roslib, rospy, time, random

from kvorum2 import gdic
from kvorum2 import tmurobot as tmu

from kvorum2.env import TEnv
from kvorum2.agent import TAgent, TSensor

###################################################################

Title = "Set Src Demo 1.05"

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
    rate = rospy.Rate(Env.Rate) # 10 50hz

    #####################################################
    # Основной цикл
    #####################################################
    GlobalTimer = 0

    # Времена для смены направления движения
    T0 = 10   # Время ожидания (просто стоим и ничего не делаем)
    T = 200   # Каждые T тактов меняем направление
    while not rospy.is_shutdown():
        print(GlobalTimer)
        # Цикл по всем агентам
        for r in tmu.Robots:
            # Если робот не дождался ответа на запрос сенсоров, то пропускаем ход
            if r.waits_for_sensors: continue

            # Чтение сенсорных данных. Все и сразу
            r.RequestSensors(sensors_list=None)
            # Вывод на экран
            if r.id==1: r.ShowStatus()

            # Анализ
            # ...

            # Действия
            # ...
            # r.Stop()
            # r.GoFwd()
            # r.GoBack()
            # r.GoFastLeft(self, ang=0)
            # r.GoFastRight(self, ang=0)
            # r.GoLeft(self, ang=0)
            # r.GoRight(self, ang=0)
            #
            if r.id==1:
                if GlobalTimer in range(T0, T0+T):
                    r.GoFwd()
                elif GlobalTimer in range(T0+T, T0+T*2):
                    r.GoLeft()
                elif GlobalTimer in range(T0+T*2, T0+T*3):
                    r.GoFastRight(1)
                elif GlobalTimer>T0+T*3:
                    print("-- GoFwd")
                    r.GoFwd()
                else:
                    print("-- Stop")
                    r.Stop()

            # Через некоторое время меняем IR-код
            if GlobalTimer % 200 == 0:
                mycode = r.GetSrc(gdic.LEVEL_IR)
                r.SetSrc(gdic.LEVEL_IR, mycode+10)

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
