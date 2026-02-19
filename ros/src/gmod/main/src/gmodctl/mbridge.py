#!/usr/bin/env python3
# coding: utf-8
"""

Библиотека согласователей топиков mobot
Version 1.0

17.11.2024

LP 03.12.2024

"""
import sys, math
import roslib, rospy

from kvorum2 import gdic
from gmodctl import quat_euler

compass_correction = 0

cspeed = None  # м/с
rotvlin = None # м/с
rotvang = None # В градусах

cellsize = None # Размер клетки Kvorum в метрах

PrimarySensors = {
    'usonic_fwd_center': -1,
    'usonic_fwd_left': -1,
    'usonic_fwd_right': -1,
    'usonic_side_left_fwd': -1,
    'usonic_side_left_bck': -1,
    'usonic_side_right_fwd': -1,
    'usonic_side_right_bck': -1,
    'usonic_bck_center': -1,

    'usonic_fwd_center_info': (0,0), # (range_min, range_max)
    'usonic_fwd_left_info': (0,0),
    'usonic_fwd_right_info': (0,0),
    'usonic_side_left_fwd_info': (0,0),
    'usonic_side_left_bck_info': (0,0),
    'usonic_side_right_fwd_info': (0,0),
    'usonic_side_right_bck_info': (0,0),
    'usonic_bck_center_info': (0,0),

    # Счетики для обеспечения инерционности
    'usonic_fwd_center_cnt': 0,
    'usonic_fwd_left_cnt': 0,
    'usonic_fwd_right_cnt': 0,
    'usonic_side_left_fwd_cnt': 0,
    'usonic_side_left_bck_cnt': 0,
    'usonic_side_right_fwd_cnt': 0,
    'usonic_side_right_bck_cnt': 0,
    'usonic_bck_center_cnt': 0,

    'servolocator': [],
    'servolocator_info': (0,0), # (range_min, range_max)
    'servolocator_A': 0,
    'servolocator_val': 0,
    'servolocator_A1': 0,
    'servolocator_A2': 0,

    'compass': -1,
    'compass_info': (0,0),
    'lidar':[],
    'arucolocator':[],
    'aruco_camera_angle': 0,
    'goal': None }


def Init():
    global compass_correction
    global cspeed, rotvlin, rotvang
    global cellsize

    compass_correction = int(rospy.get_param("~compass", 0))

    cspeed = rospy.get_param("~cspeed", 1)
    rotvlin = rospy.get_param("~rotvlin", 0.05)
    rotvang = rospy.get_param("~rotvang", 5)
    cellsize = rospy.get_param("~cellsize", 1)

    print(f":: compass_correction={compass_correction}")
    print(f":: cspeed={cspeed}, rotvlin={rotvlin}, rotvang={rotvang} cellsize={cellsize}")

################################################################################
#
#
#
#################################################################################
#
# Сообщения от ArUco-сервера
# Возвращает расстояние в сантиметрах
#
def data_superlocator_cb(msg):
    if len(msg.data)==0:
        PrimarySensors['arucolocator'] = []
        PrimarySensors['aruco_camera_angle'] = 0
        return
    data_superlocator = list(msg.data[0].data)
    datalen = len(data_superlocator)
    PrimarySensors['arucolocator'] = gdic.Vector2SuperVector(data_superlocator)
    PrimarySensors['aruco_camera_angle'] = datalen//2

#
# Инерциальный датчик
#
def mobot_imu_cb(msg):
    x, y, z, w = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
    r, p, y = quat_euler.quaternion_2_euler1(x, y, z, w)
    r = quat_euler.r2a(r)
    p = quat_euler.r2a(p)
    wcomp = quat_euler.r2a(y)

    mcomp = (360 - wcomp) + compass_correction
    mcomp = mcomp % 360
    if mcomp<0: mcomp = 360 - mcomp
    PrimarySensors['compass'] = mcomp

#
# Лидар
# Возвращает расстояние в м.
#
def mobot_lidar_cb(msg):
    PrimarySensors['lidar'] = msg.ranges
    PrimarySensors['lidar_angle_min'] = msg.angle_min
    PrimarySensors['lidar_angle_max'] = msg.angle_max
    PrimarySensors['lidar_angle_increment'] = msg.angle_increment
    PrimarySensors['lidar_range_min'] = msg.range_min
    PrimarySensors['lidar_range_max'] = msg.range_max

#
# Серволокатор
# Исходные значения - в см.
# Будем переводить в метры.
#
def mobot_servolocator_cb(msg):
    PrimarySensors['servolocator'] = [d/10.0 for d in msg.data]
    PrimarySensors['servolocator_info'] = (msg.range_min/10, msg.range_max/10)
    PrimarySensors['servolocator_A'] = msg.A
    PrimarySensors['servolocator_val'] = msg.Val
    PrimarySensors['servolocator_A1'] = msg.A1
    PrimarySensors['servolocator_A2'] = msg.A2
    return

#
# Дальномеры
# Возвращают расстояние в м.
#
def mobot_rf_cb(msg):
    RF_COUNTER = 10 #100 Инерционность
    sname = msg.header.frame_id
    val = msg.range
    PrimarySensors[sname+"_info"] = (msg.min_range, msg.max_range)

    if val>msg.min_range and val<msg.max_range:
        PrimarySensors[sname+"_cnt"] = RF_COUNTER
        PrimarySensors[sname] = val
    else:
        if PrimarySensors[sname+"_cnt"]>0:
            PrimarySensors[sname+"_cnt"] = PrimarySensors[sname+"_cnt"]-1
        else:
            PrimarySensors[sname] = val
    # Нормализация значений
    val = PrimarySensors[sname]
    if val<msg.min_range or val>=msg.max_range: val = 0
    PrimarySensors[sname] = val

    return

################################################################################
#
# Функции для отображения
#
#################################################################################
def truncatelist(D, nc):
    N = len(D)
    if(N+22>nc): RW = nc-22
    else: RW = N
    n = int((N-RW)/2)
    if n>0: L = D[n:-n]
    elif n==0: L = D
    else: L = []
    return L, N, len(L)

#
# Lidar
#
def strLidar(ncols):
    L, N0, N1 = truncatelist(PrimarySensors['lidar'], ncols)
    s = f"lidar[{N1}/{N0}] ["
    for r in L:
        c = '.' if math.isinf(r) else '*'
        s = s+c
    s = s+"]"
    return s

#
# ArUcoLocator
# Для отображения надо переворачивать
#
def strSuperLocator(ncols):
    L, N0, N1 = truncatelist(PrimarySensors['arucolocator'], ncols)
    s = f"aruco[{N1}/{N0}] ["
    for i in range(len(L)-1, -1, -1):
        c = '.' if L[i][0]==0 else "{:X}".format(L[i][1])
        s = s+c
    s = s+"]   "
    return s

#
# ServoLocator
# Для отображения надо переворачивать
#
def strServoLocator(ncols):
    L, N0, N1 = truncatelist(PrimarySensors['servolocator'], ncols-4)
    s = f"srvloc[{N1}/{N0}] ["
    range_min, range_max = PrimarySensors['servolocator_info']
    for i in range(len(L)-1, -1, -1):
        c = '.' if L[i]<=range_min or L[i]>=range_max else '*'
        s = s+c
    s = s+"]"
    return s
