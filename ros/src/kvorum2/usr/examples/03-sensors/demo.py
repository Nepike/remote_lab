#!/usr/bin/env python3
# coding: utf-8
"""
  demo.py

  Простая демонстрационная программа 03
  Сенсоры

  06.02.2015, 17.05.2015, 11.02.2024
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

Title = "RSim Demo 1.05"

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

# "Красивый" вывод локатора
# (расстояние, код)
# Только надо перевернуть
def __str(L):
    s = ""
    for i in range(len(L)-1, -1, -1):
        s = s + ('*' if L[i]!=0 else '.')
    return s

# "Красивый" вывод суперлокатора
# (расстояние, код)
# Только надо перевернуть
def __str2(L):
    s = ""
    for i in range(len(L)-1, -1, -1):
        s = s + ('*' if L[i][1]!=0 else '.')
    return s

def main(envfilename, mapfilename, agentfilename):

    global Env, Agents

    # Инициалиизация системы
    print("Init environment... ")
    Env = TEnv(envfilename)

    print("Init map", mapfilename, "...")
    exec (open(mapfilename).read(), globals())

    print("Init agents", agentfilename, "...")
    exec (open(agentfilename).read(), globals())

    InitSystem('demo', envfilename, mapfilename, agentfilename)

    print("Start main loop")
    rate = rospy.Rate(Env.Rate) # 10 50hz

    #####################################################
    # Основной цикл
    #####################################################

    while not rospy.is_shutdown():


        # Цикл по всем агентам
        for r in tmu.Robots:

            # Проверка, жив ли агент
            if not r.agent.alive: continue

            # Если робот не дождался ответа на запрос сенсоров, то пропускаем ход
            if r.waits_for_sensors: continue

            # Чтение сенсорных данных. Все и сразу
            r.RequestSensors(sensors_list=None)

            # Вывод на экран
            #r.ShowStatus()
            #
            # Извлекаем сенсорные данные
            #

            # Датчики препятствий
            print(f"-- RANGEF = {r.GetSensor('RF_FWD_LEFT')} {r.GetSensor('RF_FWD_RIGHT')} {r.GetSensor('RF_SIDE_FWD_LEFT')} {r.GetSensor('RF_SIDE_FWD_RIGHT')} {r.GetSensor('RF_SIDE_BACK_LEFT')} {r.GetSensor('RF_SIDE_BACK_RIGHT')}")

            # TSOP
            print(f"-- TSOPS: F={r.GetSensor('TSOP_FWD')} B={r.GetSensor('TSOP_BACK')} L={r.GetSensor('TSOP_LEFT')} R={r.GetSensor('TSOP_RIGHT')}")

            #
            # Локатор
            #
            print(f"-- LOC = {__str(r.GetSensor('locator'))}")

            #
            # Суперлокатор
            #
            print(f"-- SUPERLOC = {__str2(r.GetSensor('superlocator'))}")


            # Точечный датчик (радиус=0)
            print(f"-- POINTSENS = {r.GetSensor('IR_DETECTOR')}")

            #
            # Фиктивные данные (положение робота)
            #
            pos = r.GetSensor('position')
            a = pos[2]
            print(f"-- POS = {pos} A = {a}")

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
