#!/usr/bin/env python3
# coding: utf-8
"""
  demo.py

  Простая демонстрационная программа
    -- r.SetState()
    -- r.SetCurrVSpeed()
    -- r.SetSrc()
    -- tmu.SetAgentPos()
    -- tmu.KillAgent()
    
  Проблема:
    kvorum_m не иногда успевает отреагировать на изменения сотояния агентов
    (когда загрузка и отрисовка агентов происходит очень медленно, а основная программа уже изменила состояния агентов)

  06.02.2015, 17.05.2015
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

# "Красивый" вывод суперлокатора
# (расстояние, код)
def __str(L):
    s = ""
    for e in L:
        if e[1]!=0: s=s+'*'
        else: s=s+'.'
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
    rate = rospy.Rate(Env.Rate)

    #####################################################
    # Основной цикл
    #####################################################
    GlobalTimer = 0

    # Времена для смены направления движения
    T0 = 10   # Время ожидания (просто стоим и ничего не делаем)
    T = 200   # Каждые T тактов меняем направление
    dead_list = []
    while not rospy.is_shutdown():

        L=""
        num_alive = 0
        for r in tmu.Robots:
            if r.agent.alive:
                num_alive+=1
                L = L+'+'
            else:
                L = L+'-'

        print(f"{GlobalTimer} / {num_alive}/{len(tmu.Robots)} {L} Killed: {dead_list}")

        # Цикл по всем агентам
        for r in tmu.Robots:

            # Проверка, жив ли агент
            if not r.agent.alive: continue

            # Если робот не дождался ответа на запрос сенсоров, то пропускаем ход
            if r.waits_for_sensors: continue

            # Чтение сенсорных данных
            # Нас интересует только MINE_DETECTOR
            r.RequestSensors(sensors_list=['MINE_DETECTOR'])

            # Выводим на экран только необходимое
            # r.ShowStatus()

            # Анализ
            # ...

            # Действия
            # Движение по заданные линейной и угловой скоростям
            #   r.SetVSpeed(vlin, vang)
            #
            # Случайное блуждание
            if GlobalTimer%100 == 0:
                vlin = 0.75 + random.gauss(0, 0.25)
                vang = 0 + random.gauss(0, 5.0)
                r.SetCurrVSpeed(vlin, vang)

            # Через некоторое время меняем IR-код
            if GlobalTimer % 201 == 0:
                mycode = r.GetSrc(gdic.LEVEL_IR)
                r.SetSrc(gdic.LEVEL_IR, mycode+10)

            # Через некоторое время меняем состояние (цвет) агентов
            # Состояния агента: gdic (STAT_NORM, STAT_LEADER, STAT_S0, STAT_S1, STAT_S2) = range(5)
            if GlobalTimer % 1001 == 0:
                curr_state = r.agent.State
                curr_state += 1
                if curr_state>gdic.STAT_S2: curr_state = gdic.STAT_NORM
                r.SetState(curr_state)

            # Через некоторое время меняем положение агентов
            if GlobalTimer % 20001 == 0:
                x, y, a = r.agent.pos
                x += random.randint(10, 50)
                y += random.randint(10, 50)
                a += random.randint(1, 20)
                if x>Env.DIM_X: x = Env.DIM_X-1
                if y>Env.DIM_Y: y = Env.DIM_Y-1
                tmu.SetAgentPos(r.agent.id, x, y, a)

            # Наступили на мину. Агент погибает.
            c = r.GetSensor('MINE_DETECTOR')
            if c==100:
                print(f"kill {r.agent.id}")
                dead_list.append(r.agent.id)
                tmu.KillAgent(r.agent.id)
                time.sleep(0.1)

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
