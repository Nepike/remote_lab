#!/usr/bin/env python3
# coding: utf-8
"""
  main.py
  KVORUM2 MAS - MODEL
  Author: Valery Karpov
  06.02.15, 12.06.2021, 20.01.2024
  Version 2.3
  LP 14.06.2025

"""

import os, sys, time, random, math
import roslib, rospy

from kvorum2 import gdic
from kvorum2.env import TEnv
from kvorum2.agent import TAgent, TSensor

# Топики ROS
from msg_kvorum2.msg import action
from msg_kvorum2.msg import sdata, sensors
from msg_kvorum2.msg import viz as viz_proc
from msg_kvorum2.msg import pos as viz_pos

###################################################################
Title = "Kvorum2-M 2.3"

GlobalTimer = 0
Env = None
Agents = []

SERVER_ADDR = 100

pub_viz = None
pub_sens = None

###################################################################

deleted_agents=[]

def Afind(id):
    for a in Agents:
        if(a.id == id): return a
    print("AFind id=", id)
    gdic.error("agent not found")

def rsaction_callback(cmdmsg):
    cmd = cmdmsg.action
    aid = cmdmsg.agent_id

    slist = cmdmsg.slist

    arg1 = cmdmsg.arg1
    arg2 = cmdmsg.arg2
    arg3 = cmdmsg.arg3

    if(cmd != gdic.CMD_SET_CURR_V_SPEED and cmd != gdic.CMD_SET_NORM_V_SPEED):
        arg1 = int(arg1)
        arg2 = int(arg2)
        arg3 = int(arg3)

    if cmd != gdic.CMD_F_SET_FIELD:
        gdic.MY_ADDR = aid
    else:
        gdic.MY_ADDR = 0

    uncknowncommand = False

    if cmd != gdic.CMD_F_SET_FIELD:
        A = Afind(aid)
        norm_movespeed, norm_rotatespeed = A.GetNormVSpeed()
    #
    # Управление
    #
    # Движение
    #

    if(cmd == gdic.CMD_SET_CURR_V_SPEED):
        A.SetCurrVSpeed(arg1, arg2)
    elif(cmd == gdic.CMD_SET_NORM_V_SPEED):
        A.SetNormVSpeed(arg1, arg2)
    elif(cmd == gdic.CMD_FWD):
        A.SetCurrVSpeed(norm_movespeed, 0)
    elif(cmd == gdic.CMD_BACK):
        A.SetCurrVSpeed(-norm_movespeed, 0)
    elif(cmd == gdic.CMD_SET_CURR_2W_SPEED):
        A.SetCurr2WSpeed(arg1, arg2)
    elif(cmd == gdic.CMD_LEFT or cmd == gdic.CMD_FAST_LEFT):
        if(arg1==0): # Постоянный разворот
            A.SetCurrVSpeed(0, norm_rotatespeed)
        else: # Поворот на угол
            A.SetCurrVSpeed(0, 0)
            A.Turn2(arg1)
    elif(cmd == gdic.CMD_RIGHT or cmd == gdic.CMD_FAST_RIGHT):
        if(arg1==0): # Постоянный разворот
            A.SetCurrVSpeed(0, -norm_rotatespeed)
        else: # Поворот на угол
            A.SetCurrVSpeed(0, 0)
            A.Turn2(-arg1)
    elif(cmd == gdic.CMD_STOP):
        A.SetCurrVSpeed(0, 0)
    elif(cmd == gdic.CMD_DELETE_OBJ): # Пробуем убрать поле вокруг агента a на слое arg1
        EraseField(A, arg1)

    elif(cmd == gdic.CMD_F_DELETE_AGENT): # Удалить агента. Аргумент: id агента
        A.Delete()
        deleted_agents.append(aid)

    elif(cmd == gdic.CMD_F_SET_APOS): # Установить координаты агента. Аргументы: id агента, x, y, angle
        newpos = (arg1, arg2, arg3)
        A.MoveTo(newpos)

    elif(cmd == gdic.CMD_F_SET_A_SRC): # Агент становится источником сигнала value на уровне level
        A.SetSrc(arg1, arg2)              # level, val

    elif(cmd == gdic.CMD_F_SET_FIELD): # Установить значение поля. Аргументы: arg1=x, arg2=y, arg3=level, id=value
        SetField(arg1, arg2, arg3, aid)

    elif(cmd == gdic.CMD_F_SET_STATE): # Установить статус агента. Аргумент: arg1=mode
        SetState(aid, arg1)
    #
    # Запрос сенсоров
    #
    elif(cmd == gdic.CMD_GET_SENS):
        # slist содержит имена запрашиваемых сенсоров
        msg_sens = sensors()
        msg_sens.data = []
        msg_sens.src = SERVER_ADDR
        msg_sens.dest = aid #gdic.MY_ADDR
        msg_sens.tm = GlobalTimer # rospy.get_rostime() # time(NULL)

        # Читаем по запросу
        # На самом деле читать будем не здесь
        A.req_sens_list = slist

        for sname in slist:
            s = A.FindSensor(sname)
            if not s: gdic.error(f"mmain: Sensor \'{sname}\' not found")
            sensdata = A.GetSensor(sname)
            if s.valtype==gdic.RST_SUPER_VECTOR:
                sensdata = gdic.SuperVector2Vector(sensdata, 1)
            if type(sensdata)!=list: sensdata = [sensdata]
            sval = sdata()
            sval.args = s.name
            sval.data = sensdata
            msg_sens.data.append(sval)
        pub_sens.publish(msg_sens)

################################################################################
# MAIN
################################################################################

# Отправка команды инициализации всех агентов
def InitAllAgents():
    print("Init all agents... ")
    for a in Agents:
        a.req_sens_list = None
    # Инициализируем агентов для рисования
    msg = viz_proc()
    msg.cmd = gdic.VIZ_CMD_AINIT
    for a in Agents:
        d = viz_pos()
        d.id = a.id
        d.x, d.y, d.a = int(a.pos[0]), int(a.pos[1]), int(a.pos[2])
        d.cs = a.size
        d.shape = a.shape
        d.traceOn = a.traceOn
        d.show_id = a.show_id
        msg.data.append(d)
    pub_viz.publish(msg)
    print("Done")

# Отправка команды рисования всех агентов
def SendDrawAgentsCommand():
    global GlobalTimer

    msg = viz_proc()
    msg.cmd = gdic.VIZ_CMD_ADRAW
    msg.tm = GlobalTimer # rospy.get_rostime() # time(NULL)
    for a in Agents:
        #if not a.alive: continue
        d = viz_pos()
        d.id =  a.id
        d.x, d.y, d.a = int(a.pos[0]), int(a.pos[1]), int(a.pos[2])
        d.traceOn = a.traceOn
        d.alive = a.alive
        msg.data.append(d)
    pub_viz.publish(msg)

def EraseField(a, level):
    global GlobalTimer
    msg = viz_proc()
    msg.cmd = gdic.VIZ_CMD_SET_FIELD
    msg.tm = GlobalTimer # rospy.get_rostime() # time(NULL)
    x = int(a.pos[0])
    y = int(a.pos[1])
    radius = 2
    for i in range (x-radius, x+radius+1):
        for j in range (y-radius, y+radius+1):
            ix, iy = Env.normalizate_xy(i, j)
            Env.SetFieldVal(ix, iy, level, 0)
            print(f"{ix}, {iy}, {level} -> 0")
            d = viz_pos()
            d.x = ix
            d.y = iy
            d.a = level
            d.id = 0
            msg.data.append(d)
    pub_viz.publish(msg)

# Установить значение поля
def SetField(x, y, level, value):
    global GlobalTimer
    Env.SetFieldVal(x, y, level, value)
    msg = viz_proc()
    msg.cmd = gdic.VIZ_CMD_SET_FIELD
    msg.tm = GlobalTimer # rospy.get_rostime() # time(NULL)
    d = viz_pos()
    d.x = x
    d.y = y
    d.a = level
    d.id = value
    msg.data.append(d)
    pub_viz.publish(msg)

# Установить статус агента
def SetState(aid, mode):
    msg = viz_proc()
    msg.cmd = gdic.VIZ_CMD_SET_STATE
    msg.tm = GlobalTimer # rospy.get_rostime() # time(NULL)
    d = viz_pos()
    d.id = aid
    d.state = mode
    msg.data.append(d)
    pub_viz.publish(msg)

#
#
#
def main(envfilename, mapfilename, agentfilename):
    global Env, Agents, GlobalTimer
    global pub_sens, pub_viz

    ############################################################################
    #
    # Инициалиизация системы моделирования
    #
    ############################################################################

    print("Init environment... ")
    Env = TEnv(envfilename)

    print("Init map", mapfilename, "...")
    exec (open(mapfilename).read())

    print("Init agents", agentfilename, "...")
    exec (open(agentfilename).read())

    ############################################################################
    #
    # Инициалиизация ROS
    #
    ############################################################################
    rospy.init_node('kvorum_m')

    # Входной топик
    inpqlen = max(len(Agents)*7, 10)
    rospy.Subscriber(gdic.TOPIC_NAME_COMMAND, action, rsaction_callback, queue_size=inpqlen)

    # Выходные топики
    outqlen = max(len(Agents)*7, 10)
    pub_sens = rospy.Publisher(gdic.TOPIC_NAME_SENSORS, sensors, queue_size=outqlen)

    pub_viz = rospy.Publisher(gdic.TOPIC_NAME_VIZ_PROC, viz_proc, queue_size=outqlen)

    #####################################################
    #
    # Инициализируем агентов для рисования
    #
    #####################################################
    time.sleep(2)

    # Отправка команды инициализации всех агентов
    InitAllAgents()

    #####################################################
    #
    # Основной цикл
    #
    #####################################################
    print("Start main loop")
    print("Total agents (alive/dead):")

    num_agents = len(Agents)
    rpyrate = Env.Rate*num_agents
    r = rospy.Rate(rpyrate)

    GlobalTimer = 0
    dratio = 0

    curr_a_idx = 0
    while not rospy.is_shutdown():

        num_alive = 0
        for a in Agents:
            if a.alive: num_alive+=1

        a = Agents[curr_a_idx]
        if a.alive:
            a.Step()
            if a.req_sens_list:
                a.ReadSensors(a.req_sens_list)
                a.req_sens_list = None

        curr_a_idx += 1
        if curr_a_idx>=num_agents:
            curr_a_idx = 0
            GlobalTimer += 1
            print(num_alive, deleted_agents, '  \r', end='')

        #
        # Рисуем
        # Странная кратность. Хочется получить 5Гц
        dratio+=1
        if(dratio>rpyrate/5):
            dratio = 0
            SendDrawAgentsCommand()

        r.sleep()

    gdic.terminate_program()
    print("\nDone")

################################################################################
#
#
#
################################################################################
if __name__ == '__main__':

    if (len(sys.argv) < 4):
        print("\n"+Title, "\n\nUsage is:", sys.argv[0], "envfile mapfile agentfile")
        sys.exit(1)

    envfile = sys.argv[1]
    mapfile = sys.argv[2]
    agentfile = sys.argv[3]

    main(envfile, mapfile, agentfile)
