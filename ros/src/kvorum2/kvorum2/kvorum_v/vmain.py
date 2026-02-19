#!/usr/bin/env python3
# coding: utf-8
"""
  vmain.py
  KVORUM2 MAS - VIS
  Author: Valery Karpov

  06.02.2015, 18.07.2016, 03.06.2018. 11.02.2024
  Version 2.1
  LP 23.08.2024

"""
import os, sys, time, random, math
import roslib, rospy

from kvorum2 import gdic, tshapes
from vizenv import TGrEnv
from vizagent import TGrAgent

# Топики ROS
from msg_kvorum2.msg import viz as viz_proc
from msg_kvorum2.msg import pos as viz_pos

###################################################################
Title = "Kvorum_V 2.2"

Env = None
Agents = []

MSG_AINIT = None
MSG_ADRAW = None
MSG_SET_FIELD = None

###################################################################

def Afind(id):
    global Agents
    for a in Agents:
        if(a.id == id): return a
    #gdic.error("AFind: agent " + str(id) + " not found")
    print("AFind: agent " + str(id) + " not found")
    return None

def proc_callback(msg):
    global MSG_AINIT, MSG_ADRAW, MSG_SET_FIELD

    #print(f"{msg.tm} cmd: {msg.cmd}")

    if(msg.cmd==gdic.VIZ_CMD_AINIT):
        MSG_AINIT = msg
    elif(msg.cmd==gdic.VIZ_CMD_ADRAW):
        MSG_ADRAW = msg
    elif(msg.cmd==gdic.VIZ_CMD_SET_FIELD):
        MSG_SET_FIELD = msg
    elif(msg.cmd==gdic.VIZ_CMD_SET_STATE):
        for d in msg.data:
            agent = Afind(d.id)
            if(agent==None): continue
            agent.SetState(d.state) # Состояние агента

def CreateAgents():
    global MSG_AINIT
    if(MSG_AINIT == None): return
    for d in MSG_AINIT.data:
        a = TGrAgent(d.id, (d.x, d.y, d.a), d.shape, Env, d.show_id, d.cs)
        a.traceOn = d.traceOn
        a.alive = True
        a.need_redraw = True
        Agents.append(a)
    MSG_AINIT = None
    for a in Agents:
        a.Draw()

def DrawAgents():
    global MSG_ADRAW
    if(MSG_ADRAW == None): return
    for d in MSG_ADRAW.data:
        agent = Afind(d.id)
        if(agent==None): return
        agent.pos = (d.x, d.y, d.a)
        agent.traceOn = d.traceOn
        agent.alive = d.alive
        if agent.need_redraw:
            agent.Draw()
        if not agent.alive: agent.need_redraw = False
    MSG_ADRAW = None

def RedrawField():
    global MSG_SET_FIELD
    if(MSG_SET_FIELD == None): return
    for d in MSG_SET_FIELD.data:
        Env.SetFieldVal(d.x, d.y, d.a, d.id)
        Env.DrawCell(d.x, d.y)
    MSG_SET_FIELD = None

################################################################################
# MAIN
################################################################################

def main(envfilename, mapfilename):

    global Env, Agents

    #
    # Инициалиизация системы моделирования
    #
    print("Create forms... ")
    tshapes.CreateTForms(Title)

    print("Init environment... ")
    Env = TGrEnv(envfilename)

    print("Init map", mapfilename, "...")
    exec (open(mapfilename).read())

    #
    # Инициалиизация ROS
    #
    rospy.init_node('kvorum_v')
    inpqlen = max(len(Agents), 100)
    sub_act = rospy.Subscriber(gdic.TOPIC_NAME_VIZ_PROC, viz_proc, proc_callback, queue_size=inpqlen)

    #
    # Рисование
    #
    print("Draw Field... ")
    Env.DrawField()
    print("Done")

    #
    # Основной цикл
    #
    print("Start main loop")

    r = rospy.Rate(50) # 50hz
    while not rospy.is_shutdown():
        CreateAgents()
        DrawAgents()
        RedrawField()
        r.sleep()

    gdic.terminate_program()

################################################################################
#
################################################################################
if __name__ == '__main__':

    if (len(sys.argv) < 3):
        print("\n"+Title, "\n\nUsage is:", sys.argv[0], "envfile mapfile")
        sys.exit(1)

    envfile = sys.argv[1]
    mapfile = sys.argv[2]

    main(envfile, mapfile)

    mainloop()
