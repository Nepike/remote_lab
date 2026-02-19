#!/usr/bin/env python
# coding: utf-8
"""
  demo.py

  Простая демонстрационная программа
  Робот, оснащенный всевозможными датчиками

  06.02.15
  Version 1.04
  LP 17.05.2015

"""
import os, sys, time
import roslib, rospy, time, random

sys.path.append("../../pylib")
sys.path.append("src/kvorum/pylib")

import gdic, rcproto, tmurobot as tmu
from env import TEnv
from agent import TAgent, TSensor

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

    # Входной топик
    # Подписываемся на топик ardans, второй параметр - объем кэша отправки
    inpqlen = len(Agents)*7
    rospy.Subscriber("/ardans_topic", tmu.ans, tmu.rsans_callback, queue_size=inpqlen)

    # Выходной топик
    outqlen = len(Agents)*7
    pub_cmd = rospy.Publisher("/actions_topic", tmu.action, queue_size=outqlen)

    # Создаем роботов
    for a in Agents:
        r = tmu.TRobot(a, pub_cmd)
        tmu.Robots.append(r)

################################################################################
# MAIN
################################################################################

def main(envfilename, mapfilename, agentfilename):

    global Env, Agents, GlobalTimer

    # Инициалиизация системы
    print "Init environment... "
    Env = TEnv(envfilename)

    print "Init map", mapfilename, "..."
    exec (open(mapfilename).read())

    print "Init agents", agentfilename, "..."
    exec (open(agentfilename).read())

    InitSystem('demo', envfilename, mapfilename, agentfilename)

    print "Start main loop"
    rsp = rospy.Rate(5) # 50hz

    #####################################################
    # Основной цикл
    #####################################################

    while not rospy.is_shutdown():
        print GlobalTimer
        for R in tmu.Robots:
            if not R.agent.alive: continue
            R.RequestAllSensors(immediate = False, req_main_sensors=True, req_locator=False, req_super_locator=False, req_i2cdata=False, req_tsoprc5=False, req_registers=False)  
            R.ShowStatus(show_main_sensors=True, show_locator=False, show_super_locator=False, show_dataserver=False, show_tsoprc5=False, show_registers=False)
            act, arg = tmu.PROC_GOFWD, 0

            #LIM = SHARP_DIST/2
            LIM = 40

            if(sGetReflex(R)!=0):
                act, arg = tmu.PROC_GOBACK, 0
                print "*"
            elif(sGetFL(R)>LIM and sGetFR(R)>LIM):
                act, arg = tmu.PROC_GOLEFT, 10 + random.randint(-2, 2)
            elif(sGetFL(R)>LIM):
                act, arg = tmu.PROC_GORIGHT, 5 + random.randint(-1, 1)
            elif(sGetFR(R)>LIM):
                act, arg = tmu.PROC_GOLEFT, 5 + random.randint(-1, 1)

            if(GlobalTimer % 1 == 0):
                R.Make(act, arg)

        GlobalTimer += 1

        rsp.sleep()

    gdic.terminate_program()

################################################################################
#
################################################################################
if __name__ == '__main__':

    if (len(sys.argv) < 4):
        print "\n", Title, "\n\nUsage is:", sys.argv[0], "envfile mapfile agentfile"
        sys.exit(1)

    envfile = sys.argv[1]
    mapfile = sys.argv[2]
    agentfile = sys.argv[3]

    main(envfile, mapfile, agentfile)
