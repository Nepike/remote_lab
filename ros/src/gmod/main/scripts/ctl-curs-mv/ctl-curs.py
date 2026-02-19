#!/usr/bin/env python3
# coding: utf-8
"""

Ручное управление

Version 1.07

06.03.2021, 03.07.2021, 11.02.2023
LP 29.04.2024

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

from msg_yy.msg import sens as yy_sens
from msg_yy.msg import cmd as yy_cmd
from msg_yy.msg import servolocator
from msg_ans.msg import ans as yy_aruco

from geometry_msgs.msg import Twist

from sensor_msgs.msg import Joy as TJoystick

from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan

import brain
from gmodctl import mobot_models as models

Title = "\nYY Ctl 1.07\n"

pub_vel = None
pub_cmd = None

Sensors = {
    'usonic_fwd_center': -1,
    'usonic_fwd_left': -1,
    'usonic_fwd_right': -1,
    'usonic_side_left_fwd': -1,
    'usonic_side_left_bck': -1,
    'usonic_side_right_fwd': -1,
    'usonic_side_right_bck': -1,

    'usonic_fwd_center_info': (0,0), # (range_min, range_max)
    'usonic_fwd_left_info': (0,0),
    'usonic_fwd_right_info': (0,0),
    'usonic_side_left_fwd_info': (0,0),
    'usonic_side_left_bck_info': (0,0),
    'usonic_side_right_fwd_info': (0,0),
    'usonic_side_right_bck_info': (0,0),

    'usonic_fwd_center_cnt': 0,
    'usonic_fwd_left_cnt': 0,
    'usonic_fwd_right_cnt': 0,
    'usonic_side_left_fwd_cnt': 0,
    'usonic_side_left_bck_cnt': 0,
    'usonic_side_right_fwd_cnt': 0,
    'usonic_side_right_bck_cnt': 0,

    'servolocator': [],
    'servolocator_info': (0,0), # (range_min, range_max)
    'servolocator_A': 0,
    'servolocator_val': 0,
    'servolocator_A1': 0,
    'servolocator_A2': 0,

    'compass': -1,
    'lidar':[],
    'arucolocator':[],
    'goal': None}

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

    WinSens= CreateWindow(0,     0, wrows,         wcols, RIDSUBST(config['MAIN']['name']), border=True)
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
def pmap(a, a0, a1, v0, v1): return (a-a0)*(v1-v0)/(a1-a0) + v0
#
# Формируем область видимости объектов
#
def CreateOmega():
    Omega = []

    omega_dist = float(config['MAIN']['omega_dist'])

    #
    # Lidar
    #
    N = len(Sensors['lidar'])
    if N>0:
        angle_min = Sensors['angle_min']
        angle_max = Sensors['angle_max']
        angle_increment = Sensors['angle_increment']
        range_min = Sensors['range_min']
        range_max = Sensors['range_max']

        a = angle_min
        TIP = -1
        for r in Sensors['lidar']:
            if (not math.isinf(r)) and (r>range_min) and (r<range_max) and (r<omega_dist):
                x = r*math.cos(a)
                y = r*math.sin(a)
                pos = (x, y)
                Omega.append((pos, TIP))
            a += angle_increment
        # Убираем лишнее
        Omega = Omega[10:-10]

    #
    # ArUco
    #
    N = len(Sensors['arucolocator'])
    if N>0:
        # Пытаемся разобраться с геометрией
        acamangle = math.radians(Sensors['aruco_camera_angle'])
        angle_min = -acamangle/2
        angle_max = acamangle/2
        angle_increment = acamangle/N
        a = angle_min
        for e in Sensors['arucolocator']:
            r = e[0]/10 # Все в метрах
            TIP = e[1]
            if r>0:
                x = r*math.cos(a)
                y = r*math.sin(a)
                pos = (x, y)
                Omega.append((pos, TIP))
            a += angle_increment

    #
    # Дальномеры
    #
    Omega = []
    TIP = -1
    angles = {'usonic_fwd_center': 0,
              'usonic_fwd_left': -30,      'usonic_fwd_right': 30,
              'usonic_side_left_fwd': 60,  'usonic_side_right_fwd': -60,
              'usonic_side_left_bck': 120, 'usonic_side_right_bck': -120}
    for name in angles.keys():
        r = Sensors[name]
        min_range, max_range = Sensors[name+"_info"]
        if (r>min_range) and (r<max_range) and  (r<omega_dist):
            a = math.radians(angles[name])
            x = r*math.cos(a)
            y = r*math.sin(a)
            pos = (x, y)
            if x>0: Omega.append((pos, TIP))

    #
    # ServoLocator
    #
    Omega = []
    TIP = -1
    Data = Sensors['servolocator']
    N = len(Data)
    range_min, range_max = Sensors['servolocator_info']
    for i in range(10, N-10):
        r = Data[i]
        if r<=range_min or r>=range_max:
            continue
        a = pmap(i, 0, N-1, Sensors['servolocator_A1'], Sensors['servolocator_A2'])
        a = math.radians(a)
        x = r*math.cos(a)
        y = r*math.sin(a)
        pos = (x, y)
        Omega.append((pos, TIP))

    return Omega

################################################################################
#
#
################################################################################

def sens_str(title, key):
    return "{:>10}: {:4.2f}  ".format(title, Sensors[key])

def ShowSensors():
    global AutoRegime

    def truncatelist(D, nc):
        N = len(D)
        if(N+22>nc): RW = nc-22
        else: RW = N
        n = int((N-RW)/2)
        if n>0: L = D[n:-n]
        elif n==0: L = D
        else: L = []
        return L, N, len(L)

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
    WinSens.addstr(y, x, f"goal: {Sensors['goal']}")
    y+=1
    WinSens.addstr(y, x, "---------------------------")
    y+=1

    _, ncols = WinSens.getmaxyx()

    #
    # Lidar
    #
    L, N0, N1 = truncatelist(Sensors['lidar'], ncols)
    s = f"lidar[{N1}/{N0}] ["
    for r in L:
        c = '.' if math.isinf(r) else '*'
        s = s+c
    s = s+"]"
    WinSens.addstr(y, x, s)
    y+=1

    #
    # ArUcoLocator
    #
    L, N0, N1 = truncatelist(Sensors['arucolocator'], ncols)
    s = f"aruco[{N1}/{N0}] ["
    for r in L:
        c = '.' if r[0]==0 else "{:X}".format(r[1])
        s = s+c
    s = s+"]   "
    WinSens.addstr(y, x, s)
    y+=1

    #
    # ServoLocator
    # Для отображения надо переворачивать
    L, N0, N1 = truncatelist(Sensors['servolocator'], ncols)
    range_min, range_max = Sensors['servolocator_info']
    s = f"srvloc[{N1}/{N0}] ["
    for i in range(len(L)-1, -1, -1):
        c = '.' if L[i]<=range_min or L[i]>=range_max else '*'
        s = s+c
    s = s+"]   "
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

def aruco_cb(msg):
    rcproto_POS_LEN = 5
    rcproto_POS_DATA = 6
    data = list(msg.data)
    data_superlocator = []

    # Нормализация значений
    datalen = data[rcproto_POS_LEN]
    for i in range(datalen//2): 
        idx = rcproto_POS_DATA+i*2
        data_superlocator.append((data[idx], data[idx+1]))

    Sensors['arucolocator'] = data_superlocator
    Sensors['aruco_camera_angle'] = msg.size
    return

def rf_cb(msg):
    global Sensors
    sname = msg.header.frame_id
    val = msg.range
    Sensors[sname+"_info"] = (msg.min_range, msg.max_range)

    if val>msg.min_range and val<msg.max_range:
        Sensors[sname+"_cnt"] = 100
        Sensors[sname] = val
    else:
        if Sensors[sname+"_cnt"]>0:
            Sensors[sname+"_cnt"] = Sensors[sname+"_cnt"]-1
        else:
            Sensors[sname] = val
    return

def imu_cb(msg):
    global Sensors
    x, y, z, w = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
    r, p, y = quat_euler.quaternion_2_euler1(x, y, z, w)
    r = quat_euler.r2a(r)
    p = quat_euler.r2a(p)
    y = quat_euler.r2a(y)
    Sensors['compass'] = y
    return

def lidar_cb(msg):
    Sensors['lidar'] = msg.ranges
    Sensors['angle_min'] = msg.angle_min
    Sensors['angle_max'] = msg.angle_max
    Sensors['angle_increment'] = msg.angle_increment
    Sensors['range_min'] = msg.range_min
    Sensors['range_max'] = msg.range_max
    return

def servolocator_cb(msg):
    Sensors['servolocator'] = msg.data
    Sensors['servolocator_info'] = (msg.range_min, msg.range_max)
    Sensors['servolocator_A'] = msg.A
    Sensors['servolocator_val'] = msg.Val
    Sensors['servolocator_A1'] = msg.A1
    Sensors['servolocator_A2'] = msg.A2
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
    global Sensors
    global AutoCtl

    global WinSens, WinLog, Screen

    global config
    global _RID_

    _RID_ = rid
    config = configparser.ConfigParser()
    config.read(configfile)

    brain.print_log = print_log
    models.print_log = print_log

    print(config.sections())
    for s in config.sections():
        print(s)
        for e in config[s]:
            print("    ", e, "=", config[s][e])

    brain.Init()

    rospy.init_node('ctl_node')

    pose_sub = rospy.Subscriber('yy_sensors', yy_sens, data_cb)

    aruco_sub = rospy.Subscriber(RIDSUBST(config['EXTTOPICS']['arucolocator']), yy_aruco, aruco_cb)

    joystik_sub = rospy.Subscriber('joy', TJoystick, joy_cb)

    imu_sub = rospy.Subscriber(RIDSUBST(config['INPTOPICS']['imutopic']), Imu, imu_cb)
    lidar_sub = rospy.Subscriber(RIDSUBST(config['INPTOPICS']['lidar']), LaserScan, lidar_cb)

    for r in config.items('RANGETOPICS'):
        rospy.Subscriber(RIDSUBST(r[1]), Range, rf_cb)

    servolocator_sub = rospy.Subscriber(RIDSUBST(config['SERVOLOCATOR']['locator']), servolocator, servolocator_cb)

    # Выходные топики
    pub_cmd = rospy.Publisher('yy_command', yy_cmd, queue_size = 1)
    pub_vel = rospy.Publisher(RIDSUBST(config['OUTTOPICS']['veltopic']), Twist, queue_size = 1)

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

        Omega = CreateOmega()

        # Status
        WinSens.addstr(1, 1, f"AutoRegime={AutoRegime} |Omega|={len(Omega)} Reflex={brain.RobotState.reflexlevel}    ")

        key = WinLog.getch()
        if not EvalKey(key): break

        cmd = brain.MakeStep(Sensors, Omega)
        print_log("-- cmd={:<20} ({:6.3f}, {:6.3f})".format(brain.RobotState.cmd[0], brain.RobotState.cmd[1][0], brain.RobotState.cmd[1][1]))
        if brain.RobotState.goalpos:
            print_log("   GOAL: a={:<6.2f} r={:<6.2f} tip={:02X}".format(brain.RobotState.goalpos[0], brain.RobotState.goalpos[1], brain.RobotState.goalpos[2]))
        if AutoRegime:
            ExecuteCmd(cmd)

        rate.sleep()

    curses.endwin()
    GoStop()

################################################################################
#
#
#
################################################################################
if __name__ == "__main__" :

    print(Title)

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
