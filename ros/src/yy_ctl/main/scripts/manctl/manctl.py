#!/usr/bin/env python3
# coding: utf-8

"""
Version 1.2

06.03.2021, 03.07.2021, 11.02.2023, 18.09.2023

LP 23.01.2025

Ручное управление

input:  msg_yy.msg.sens, geometry_msgs.msg.Twist sensor_msgs.msg.Joy
output: msg_yy.msg.cmd

"""
#from __future__ import division

import time
import sys, rospy
from threading import Thread

sys.path.insert(0, "../lib")

from yyctl import y13cmd
from msg_yy.msg import sens as yy_sens
from msg_yy.msg import cmd as yy_cmd

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32 as yy_actuator
from sensor_msgs.msg import Joy as TJoystick

Title = "\nYY Ctl 1.2\n"

pub_vel = None
pub_cmd = None
pub_act = None


HelpStr = '''
Commands:
q     -- quit
? (h) -- help
w (W) -- forward
s (S) -- back
a (A) -- left
d (D) -- right
g     -- print sensors
1     -- beep ON
2     -- beep OFF

z     -- Поворачиваем актуатором влево
x     -- Выставляем актуатор в центральное положение
c     -- Поворачиваем актуатором вправо

v, b  -- Вперед/назад по прямому заданию ШИМ для двигателей (-255...+255)
V, B  -- Вперед/назад с ПИД-регулированием для двигателей (-255...+255)

'''

ProgramTerminated = False

EnableCmdVel = True

#
# Свалим все управляющие параметры в одну кучу
#
class TCtlParams:
    def __init__(self):
        self.vlinear = 0
        self.vangular = 0
        self.angle = [0,0,0]
        self.arg = [0,0,0]
        self.cmd = 0
        self.buttons = [0]*20

class ThreaCmdSender(Thread):
    def __init__(self):
        Thread.__init__(self)

    def run(self):
        while not ProgramTerminated:
            # Управление сервомашинками и прочими делами
            SendCtlCmd(pub_cmd, CmdMsg)
            # Управление скоростью
            if EnableCmdVel:
                SendVelCmd(pub_vel)
            print('\t\t\t\t\tV: {:6.3f} {:6.3f} A: {} {}'.format(CtlP.vlinear, CtlP.vangular, CmdMsg.command, CmdMsg.arg), end='')
            print(' '.rjust(30, ' ')+'\r', end='')
            time.sleep(0.1)

CtlP = TCtlParams()

VelMsg = Twist()
CmdMsg = yy_cmd()
CmdMsg.angle = [0,0,0]
CmdMsg.arg = [0,0,0]
CmdMsg.command = 0
VelAct = yy_actuator()

data_sensors = None

def data_cb(msg):
    global data_sensors
    #print(':', msg.tm, msg.data)
    #print(msg)
    data_sensors = msg
    return

def joy_cb(msg):
    global pred_w_l, pred_w_r
    CtlP.vangular = int(msg.axes[0]*1000)/1000
    CtlP.vlinear = int(msg.axes[1]*1000)/1000
    for i in range(min(len(msg.buttons),len(CtlP.buttons))):
        CtlP.buttons[i] = msg.buttons[i]

    EnableCmdVel = True
    CmdMsg.command = -1
    SendVelCmd(pub_vel)
    return

def readstr(prompt):
    if sys.version_info.major<3:
        s = raw_input(prompt)
    else:
        s = input(prompt)
    return s

# Управление скоростью
def SendVelCmd(pub):
    global VelMsg
    if (CtlP.vangular != VelMsg.angular.z) or (CtlP.vlinear != VelMsg.linear.x) or True:
        VelMsg.linear.x = CtlP.vlinear
        VelMsg.angular.z = CtlP.vangular
        # Публикуем
        pub.publish(VelMsg)

# Управление сервомашинками и прочими устройствами
# Публикуем
def SendCtlCmd(pub, msg):
    pub.publish(msg)

#
#
#
def main():
    global pub_vel, pub_cmd, EnableCmdVel, pub_act

    print(Title)

    rospy.init_node('yy_node')

    pose_sub = rospy.Subscriber('yy_sensors', yy_sens, data_cb)

    joystik_sub = rospy.Subscriber('joy', TJoystick, joy_cb)

    pub_cmd = rospy.Publisher('yy_command', yy_cmd, queue_size = 1)
    pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    pub_act = rospy.Publisher('act_cmd', yy_actuator, queue_size = 1)

    CmdSender = ThreaCmdSender()
    CmdSender.setDaemon(True)
    CmdSender.start()

    rate = rospy.Rate(50) # 100 Гц

    while not rospy.is_shutdown():
        VelAct = None
        cmd_ok = False
        s = readstr("  <cmd (q - quit, ? - help)>")
        if s in ['Q', 'q']:
            CtlP.vlinear, CtlP.vangular = 0, 0
            SendVelCmd(pub_vel)
            time.sleep(0.5)
            ProgramTerminated = True
            break
        if s in ['?', 'h', 'help']: print(HelpStr)

        #
        # Движение по ПИД-регулятору
        #
        elif s in ['w']: CtlP.vlinear, CtlP.vangular =  0.5,  0
        elif s in ['W']: CtlP.vlinear, CtlP.vangular =  1, 0
        elif s in ['s']: CtlP.vlinear, CtlP.vangular = -0.5,  0
        elif s in ['S']: CtlP.vlinear, CtlP.vangular = -1, 0
        elif s in ['a']: CtlP.vlinear, CtlP.vangular =  0,    1
        elif s in ['A']: CtlP.vlinear, CtlP.vangular =  0.1,  0.05
        elif s in ['d']: CtlP.vlinear, CtlP.vangular =  0,   -1
        elif s in ['D']: CtlP.vlinear, CtlP.vangular =  0.1, -0.05
        elif s in [' ']: CtlP.vlinear, CtlP.vangular =  0,    0

        #
        # Прямое задание скоростей (ШИМ) вращения (без ПИД-регулятора)
        #
        elif s in ['v']:
            CmdMsg.command = y13cmd.Y13_CMD_DCTL
            CmdMsg.arg[0], CmdMsg.arg[1]  = 100, 100
            EnableCmdVel = False
            cmd_ok = True
        elif s in ['b']:
            CmdMsg.command = y13cmd.Y13_CMD_DCTL
            CmdMsg.arg[0], CmdMsg.arg[1]  = -100, -100
            EnableCmdVel = False
            cmd_ok = True
        elif s in ['V']:
            CmdMsg.command = y13cmd.Y13_CMD_PIDCTL
            CmdMsg.arg[0], CmdMsg.arg[1]  = 100, 100
            EnableCmdVel = False
            cmd_ok = True
        elif s in ['B']:
            CmdMsg.command = y13cmd.Y13_CMD_PIDCTL
            CmdMsg.arg[0], CmdMsg.arg[1]  = -100, -100
            EnableCmdVel = False
            cmd_ok = True
        #
        # Сенсорика
        #
        elif s in ['g']:
            print("------------------------")
            print(data_sensors)
            print("------------------------")
        #
        # Эффекторы
        #
        elif s in ['1']:
            CmdMsg.command = y13cmd.Y13_CMD_BEEP_ON
            cmd_ok = True
        elif s in ['2']:
            CmdMsg.command = y13cmd.Y13_CMD_BEEP_OFF
            cmd_ok = True
        elif s in ['3']:
            CmdMsg.command = y13cmd.Y13_CMD_SET_ENC
            CmdMsg.arg[0] = 0
            CmdMsg.arg[1] = 0
            cmd_ok = True
        #
        # RC5
        #
        elif s in ['8']:
            CmdMsg.command = y13cmd.Y13_CMD_USR
            CmdMsg.da[0] = y13cmd.subcmdSET_RC5 # Параметры маяка RC5
            CmdMsg.da[1] = 0x12345678           # Передаваемый код RC5
            CmdMsg.da[2] = 32                   # Количество разрядов в пакете RC5 (не более 32)
            CmdMsg.da[3] = 3                    # Частота генерации пакетов, Гц
            cmd_ok = True
        elif s in ['9']:
            CmdMsg.command = y13cmd.Y13_CMD_USR
            CmdMsg.da[0] = y13cmd.subcmdSET_RC5
            CmdMsg.da[1] = 0
            CmdMsg.da[2] = 0
            CmdMsg.da[3] = 0
            cmd_ok = True
        #
        # Управление актуатором
        #
        elif s in ['z']: VelAct = 240 # Поворачиваем актуатором влево
        elif s in ['x']: VelAct = 190 # Выставляем актуатор в центральное положение
        elif s in ['c']: VelAct = 140 # Поворачиваем актуатором вправо

        if VelAct:
            pub_act.publish(VelAct)
        #
        # Взаимные блокировки двигательных команд
        #
        if s in ['w', 'W', 's', 'S', 'a', 'A', 'd', 'D', ' ']:
            CmdMsg.command = -1
            EnableCmdVel = True
        if s in ['v', 'V', 'b', 'B']:
            EnableCmdVel = False

        rate.sleep()

if __name__ == "__main__" :

    main()
