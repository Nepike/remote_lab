#!/usr/bin/env python3
# coding: utf-8
"""
  demo.py

  Простая демонстрационная программа
  Робот, оснащенный всевозможными датчиками
  Демонстрация
  -- Перенос агента (установить координаты)
     tmu.SetAgentPos()
  -- Уничтожение (поедание) объектов
     r.ProcEat()
  -- Порождение объектов
     r.ProcBirth()

  06.02.2015
  Version 1.1
  LP 14.06.2025

"""
import os, sys, time
import roslib, rospy, time, random

from kvorum2 import gdic, tmurobot as tmu
from kvorum2.env import TEnv
from kvorum2.agent import TAgent, TSensor

###################################################################

Title = "Demo 1.1"

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

################################################################################
# MAIN
################################################################################

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
    rate = rospy.Rate(Env.Rate)

    #####################################################
    # Основной цикл
    #####################################################

    while not rospy.is_shutdown():
        print(f"\r{GlobalTimer}", end='')
        for r in tmu.Robots:
            if not r.agent.alive: continue
            # Если робот не дождался ответа на запрос сенсоров, то пропускаем ход
            if r.waits_for_sensors: continue

            r.RequestSensors(sensors_list=None)

            act, arg = tmu.PROC_GOFWD, 0
            if GlobalTimer % 100 == 0: act, arg = tmu.PROC_GOLEFT, 10 + random.randint(-10, 10)

            # Нас интересует только пища (датчик цвета)
            color = r.GetSensor("color")
            # Поедаем цвет №1
            if color == 1:
                print(f"\ncolor={color}: eat")
                #act, arg = tmu.PROC_EAT, gdic.LEVEL_COLOR
                r.ProcEat(gdic.LEVEL_COLOR)

            # Порождаем объект - цвет №11
            if GlobalTimer % 100 == 0:
                birthcolor = 11
                print(f"\n-- birth color={birthcolor}")
                #act, arg = tmu.PROC_BIRTH, (gdic.LEVEL_COLOR, birthcolor)
                r.ProcBirth((gdic.LEVEL_COLOR, birthcolor))

            # Рефлексы
            if(sGetFL(r)<15 and sGetFR(r)<15):
                act, arg = tmu.PROC_GOLEFT, 10 + random.randint(-10, 10)
            elif(sGetFL(r)<15):
                act, arg = tmu.PROC_GORIGHT, 5 + random.randint(-5, 5)
            elif(sGetFR(r)<15):
                act, arg = tmu.PROC_GOLEFT, 5 + random.randint(-5, 5)

            r.Make(act, arg)

        GlobalTimer += 1

        # Установить координаты агента. Аргументы: id агента, x, y, angle
        if((GlobalTimer%5000)==0): tmu.SetAgentPos(1, 10, 10, 0)

        # Kill object example
        if(GlobalTimer==50000): tmu.KillAgent(1)

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
