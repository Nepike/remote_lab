#!/usr/bin/env python3
# coding: utf-8
"""

Ручное управление

Version 1.2

06.03.2021, 03.07.2021, 11.02.2023, 29.04.2024
LP 18.11.2024

input:  msg_yy.msg.sens, geometry_msgs.msg.Twist sensor_msgs.msg.Joy
output: msg_yy.msg.cmd

type gazebo_msgs/ContactsState
https://forum.humanbrainproject.eu/t/add-contact-sensor-or-example/550

"""

import math, time
import sys, os
import rospy
import argparse
import configparser
import curses
from signal import signal, SIGWINCH

from yyctl import y13cmd
from gmodctl import quat_euler
from gmodctl import mbridge

from msg_yy.msg import sens as yy_sens
from msg_yy.msg import cmd as yy_cmd
from msg_yy.msg import servolocator

from msg_kvorum2.msg import sdata
from msg_kvorum2.msg import sensors as yy_aruco

from geometry_msgs.msg import Twist

from sensor_msgs.msg import Joy as TJoystick

from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan

from kvorum2 import gdic

Title = "ManCtl 1.2a"

pub_vel = None
pub_cmd = None


HelpStr = '''
Commands:
q - quit
? (h) - help
w (W) - forward
s (S) - back
a (A) - left
d (D) - right
g     - print sensors
1     - beep ON
2     - beep OFF
'''

################################################################################
# Оконные дела
################################################################################
WinSens = None
WinLog = None
Screen = None

def CreateWindows():
    global WinSens, WinLog, Screen
    nrows, ncols = Screen.getmaxyx()
    wcols = ncols-2
    wrows = 16

    WinSens= CreateWindow(0,     0, wrows,         wcols, Title+' | '+RIDSUBST(config['MAIN']['name']), border=True)
    WinLog = CreateWindow(wrows, 0, nrows - wrows, wcols, '', border=False)

def resize_handler(signum, frame):
    curses.endwin()
    Screen.refresh()
    CreateWindows()

################################################################################

AutoRegime = False
config = None

################################################################################
#
# Сервисные функции
#
################################################################################

def error(msg):
    GoStop()
    curses.endwin()
    print(f"\n*** error: {msg}\n")
    sys.exit(1)

# Создание окна
def CreateWindow(y, x, rows, cols, title=None, border=True):
    w = curses.newwin(rows, cols, y, x)
    w.scrollok(True)
    w.nodelay(True)
    w.keypad(1)
    #w.bkgd(' ', curses.color_pair(1))
    if border: w.box()
    if title: w.addstr(0, 1, title, curses.color_pair(1))
    if title: w.addstr(0, 1, title)
    w.refresh()
    return w

def print_log(s):
    if not WinLog: return
    WinLog.addstr(s+"\n")
    WinLog.refresh()

################################################################################
#
# Автономное управление
#
################################################################################
_vang_slow = 0.25
_vang_fast = 1.00
_vlin_fast = 0.50
_vlin_slow = 0.25

def GoFwd(fast=True):
    if fast: VelMsg.linear.x, VelMsg.angular.z =  _vlin_fast, 0
    else:    VelMsg.linear.x, VelMsg.angular.z =  _vlin_slow, 0
    SendVelCmd(pub_vel)

def GoBack(fast=True):
    if fast: VelMsg.linear.x, VelMsg.angular.z = -_vlin_fast, 0
    else:    VelMsg.linear.x, VelMsg.angular.z = -_vlin_slow, 0
    SendVelCmd(pub_vel)

def GoLeft(fast=True):
    if fast: VelMsg.linear.x, VelMsg.angular.z =  0, _vang_fast
    else:    VelMsg.linear.x, VelMsg.angular.z =  0, _vang_slow
    SendVelCmd(pub_vel)

def GoRight(fast=True):
    if fast: VelMsg.linear.x, VelMsg.angular.z =  0, -_vang_fast
    else:    VelMsg.linear.x, VelMsg.angular.z =  0, -_vang_slow
    SendVelCmd(pub_vel)

def GoStop():
    VelMsg.linear.x, VelMsg.angular.z =  0,   0
    SendVelCmd(pub_vel)

def GoAng(a):
    VelMsg.linear.x, VelMsg.angular.z =  0.1, a
    SendVelCmd(pub_vel)

def GoTwist(vlin, vang):
    VelMsg.linear.x = vlin
    VelMsg.angular.z =  vang
    SendVelCmd(pub_vel)

################################################################################
# cmd = (name, (a1, a2))
def ExecuteCmd(cmd):
    if   cmd[0] == 'CMD_GO_BACK_FAST': GoBack(fast=True)
    elif cmd[0] == 'CMD_GO_BACK_SLOW': GoBack(fast=False)
    elif cmd[0] == 'CMD_GO_RIGHT_FAST': GoRight(fast=True)
    elif cmd[0] == 'CMD_GO_RIGHT_SLOW': GoRight(fast=False)
    elif cmd[0] == 'CMD_GO_LEFT_FAST': GoLeft(fast=True)
    elif cmd[0] == 'CMD_GO_LEFT_SLOW': GoLeft(fast=False)
    elif cmd[0] == 'CMD_GO_FWD_FAST': GoFwd(fast=True)
    elif cmd[0] == 'CMD_GO_FWD_SLOW': GoFwd(fast=False)
    elif cmd[0] == 'CMD_GO_GOAL': GoAng(cmd[1][0])
    elif cmd[0] == 'CMD_TWIST': GoTwist(cmd[1][0], cmd[1][1])
    else:
        error(f'Unknown command: {cmd}')

################################################################################
#
#
################################################################################

def sens_str(title, key):
    return "{:>10}: {:4.2f}  ".format(title, mbridge.PrimarySensors[key])

def ShowSensors():
    x = 1
    y = 2

    WinSens.addstr(y, x, sens_str('fwd_c', 'usonic_fwd_center'))
    y+=1
    WinSens.addstr(y, x, sens_str('fwd_l', 'usonic_fwd_left'))
    y+=1
    WinSens.addstr(y, x, sens_str('fwd_r', 'usonic_fwd_right'))
    y+=1
    WinSens.addstr(y, x, sens_str('sleft_fwd', 'usonic_side_left_fwd'))
    y+=1
    WinSens.addstr(y, x, sens_str('sleft_bck', 'usonic_side_left_bck'))
    y+=1
    WinSens.addstr(y, x, sens_str('sright_fwd', 'usonic_side_right_fwd'))
    y+=1
    WinSens.addstr(y, x, sens_str('sright_bck', 'usonic_side_right_bck'))
    y+=1
    WinSens.addstr(y, x, sens_str('A', 'compass'))
    y+=1
    y+=1

    _, ncols = WinSens.getmaxyx()

    # Lidar
    s = mbridge.strLidar(ncols-2)
    WinSens.addstr(y, x, s)
    y+=1

    # ArUcoLocator
    s = mbridge.strSuperLocator(ncols-2)
    WinSens.addstr(y, x, s)
    y+=1

    # ServoLocator
    s = mbridge.strServoLocator(ncols-2)
    WinSens.addstr(y, x, s)

    WinSens.refresh()

################################################################################
#
#
#
################################################################################

VelMsg = Twist()

data_sensors = None

################################################################################
#
#
#
################################################################################
def data_cb(msg):
    global data_sensors
    data_sensors = msg
    return

def joy_cb(msg):
    VelMsg.angular.z = int(msg.axes[0]*1000)/1000
    VelMsg.linear.x = int(msg.axes[1]*1000)/1000
    SendVelCmd(pub_vel)
    return

################################################################################
#
#
#
################################################################################
# Управление скоростью
def SendVelCmd(pub):
    global VelMsg
    pub.publish(VelMsg)

################################################################################

def EvalKey(key: int):
    global AutoRegime
    if key==-1: return True
    if key==27: return False

    AutoRegime = False
    if key in [ord('?'), ord('h')]: print_log(HelpStr)
    #
    # curses.KEY_UP, curses.KEY_DOWN, curses.KEY_LEFT, curses.KEY_RIGHT
    # curses.KEY_CANCEL, curses.KEY_ENTER
    # curses.KEY_PPAGE
    # curses.KEY_HOME
    #
    elif key in [ord('w'), 259]: GoFwd(fast=False)
    elif key in [ord('W'), 337]: GoFwd(fast=True)
    elif key in [ord('s'), 258]: GoBack(fast=False)
    elif key in [ord('S'), 336]: GoBack(fast=True)
    elif key in [ord('a'), 260]: GoLeft(fast=False)
    elif key in [ord('A'), 393]: GoLeft(fast=True)
    elif key in [ord('d'), 261]: GoRight(fast=False)
    elif key in [ord('D'), 402]: GoRight(fast=True)
    elif key in [ord(' ')]: GoStop()
    elif key in [ord('z'), ord('Z')]:
        AutoRegime = True

    return True

################################################################################

_RID_ = 1

def RIDSUBST(s):
  v = s.replace("{RID}", str(_RID_))
  return v

################################################################################
#
#
# rid - robot`s id
#
################################################################################
def main(configfile, rid):

    global pub_vel, pub_cmd

    global WinSens, WinLog, Screen

    global config
    global _RID_

    _RID_ = rid
    config = configparser.ConfigParser()
    config.read(configfile)

    print(config.sections())
    for s in config.sections():
        print(s)
        for e in config[s]:
            print("    ", e, "=", config[s][e])

    rospy.init_node('ctl_node')

    rospy.Subscriber('yy_sensors', yy_sens, data_cb)

    rospy.Subscriber(RIDSUBST(config['EXTTOPICS']['arucolocator']), yy_aruco, mbridge.data_superlocator_cb)

    rospy.Subscriber('joy', TJoystick, joy_cb)

    rospy.Subscriber(RIDSUBST(config['INPTOPICS']['imutopic']), Imu, mbridge.mobot_imu_cb)
    rospy.Subscriber(RIDSUBST(config['INPTOPICS']['lidar']), LaserScan, mbridge.mobot_lidar_cb)

    for r in config.items('RANGETOPICS'):
        rospy.Subscriber(RIDSUBST(r[1]), Range, mbridge.mobot_rf_cb)

    rospy.Subscriber(RIDSUBST(config['SERVOLOCATOR']['locator']), servolocator, mbridge.mobot_servolocator_cb)

    # Выходные топики
    pub_cmd = rospy.Publisher('yy_command', yy_cmd, queue_size = 1)
    pub_vel = rospy.Publisher(RIDSUBST(config['OUTTOPICS']['veltopic']), Twist, queue_size = 1)

    mbridge.Init()

    #
    # Curses
    #
    Screen = curses.initscr()
    curses.start_color()
    curses.curs_set(0)
    curses.cbreak()

    signal(SIGWINCH, resize_handler)
    CreateWindows()

    rate = rospy.Rate(50) # 100 Гц

    while not rospy.is_shutdown():

        ShowSensors()

        key = WinLog.getch()
        if not EvalKey(key): break

        rate.sleep()

    curses.endwin()
    GoStop()

################################################################################
#
#
#
################################################################################
if __name__ == "__main__" :

    print('\n'+Title+'\n')

    parser = argparse.ArgumentParser(prog = '', description = Title, epilog = '')
    parser.add_argument('--rid', nargs='?', type=int, default=1, required=False, metavar = 'robot_id', help = 'robot id (id=1,2...)')
    parser.add_argument('--cfg', nargs='?', default=None, required=True, metavar = 'filename', help = 'topics descr filename')

    # Странный финт
    # Первый и два последних аргумента (__name, __log) не нужны
    # Сделан потому, что при запуске roslaunch добавляются агрументы "__name:=...", и "__log:=..."
    arglist = []
    for e in sys.argv:
        if e.find("__name:=")==0: continue
        if e.find("__log:=")==0: continue
        arglist.append(e)
    namespace = parser.parse_args(arglist[1:])
    if namespace.rid is None: namespace.rid = 1

    print(namespace)

    main(namespace.cfg, namespace.rid)
