#!/usr/bin/env python3
# coding: utf-8
"""
  ArUco_server-2
  Программа для преследования
  ROS Version
  
  V 1.14
  12.06.2021, 30.01.2022, 15.05.2023
  LP 30.07.2023
  
  Публикует в msg_yy.msg/arucopos: "arucopos"
  
"""
import cv2 as cv
import numpy as np
import argparse, os, sys, time

import roslib, rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


from yyctl import arfunc
from msg_yy.msg import arucopos
from msg_yy.msg import arucopoint

Title = 'ArUco Server-2 1.14'

adict = cv.aruco.DICT_4X4_50

bridge = None
cv_image = None

#
#
#
def img_callback(data):
    global bridge, cv_image
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except e:
        print(e)

#
# Публикация суперлокатора
# Отправка пакета
#
def SendArucoPos(pub, arloc):
    msg = arucopos()
    msg.tm = rospy.Time.now()
    msg.points = arloc
    pub.publish(msg)
    return

######################################################################

RosFreq = None

def main(inptopicname, configfile, outtopicname, show_window, print_result):
    global bridge, cv_image
    global RosFreq

    # Загрузить словарь, который использовался для создания маркеров
    dictionary = cv.aruco.Dictionary_get(adict)

    # Инициализировать параметры детектора, используя значения по умолчанию
    parameters = cv.aruco.DetectorParameters_create()

    # Initialize the CvBridge class
    bridge = CvBridge()

    rospy.init_node('aruco_server')

    # Входной топик
    image_sub = rospy.Subscriber(inptopicname, Image, img_callback)

    # Выходной топик
    pub_arucopos = rospy.Publisher(outtopicname, arucopos, queue_size = 1)

    camera_matrix, dist_coeff = arfunc.loadCoefficients(configfile+".yaml")
    if (camera_matrix is None) or (dist_coeff is None):
        print("*** error loading config file "+configfile+".yaml")
        sys.exit(1)

    rate = rospy.Rate(RosFreq)
    while not rospy.is_shutdown():
        for i in range(len(arfunc.RLOC)):
            if arfunc.RLOC[i][2]>0: arfunc.RLOC[i][2] -= 1
            else:
                arfunc.RLOC[i][0], arfunc.RLOC[i][1] = 0, 0

        if not (cv_image is None):
            # Обнаружение маркеров на изображении
            markerCorners, markerIds, rejectedCandidates = cv.aruco.detectMarkers(cv_image, dictionary, parameters=parameters)
            rvec, tvec, _ = cv.aruco.estimatePoseSingleMarkers(markerCorners, arfunc.markerLength, camera_matrix, dist_coeff)

            if not (markerIds is None):
                if(show_window):
                    cv.aruco.drawDetectedMarkers(cv_image, markerCorners)  # Draw a square around the markers
                arfunc.Plocator2(arfunc.CAMERA_ANGLE, arfunc.NUM_PIXELS, markerIds, markerCorners, tvec)

            if print_result:
                arfunc.ShowLocator(arfunc.RLOC)

            if(show_window):
                cv.imshow("Image window", cv_image)
                key = cv.waitKey(1) # Ждём нажатия клавиши
                if key == 27:       # ESC - выход
                    break

        # Вычисляем параметры маркеров:
        # x - центр тяжести (угол, от 0 до CAMERA_ANGLE)
        # weight - количество маркеров
        # dist - среднее расстояние
        nmx = {}
        for i in range(arfunc.CAMERA_ANGLE):
            dist, id = arfunc.RLOC[i][0], arfunc.RLOC[i][1]
            if id==0: continue

            if nmx.get(id) is None:
                nmx[id] = arucopoint()
                nmx[id].id, nmx[id].x, nmx[id].weight, nmx[id].dist = id, i, 1, dist
            else:
                nmx[id].x += i
                nmx[id].weight += 1
                nmx[id].dist += dist

        for k in nmx.keys():
            nmx[k].x = int(nmx[k].x/nmx[k].weight)
            nmx[k].dist = int(nmx[k].dist/nmx[k].weight)
        print("nmx = ", nmx)

        aruco_positions = []
        for k in nmx.keys():
            aruco_positions.append(nmx[k])
        # Публикуем
        SendArucoPos(pub_arucopos, aruco_positions)

        rate.sleep()

    if(show_window): cv.destroyAllWindows() # убиваем окно

#
#
#

if __name__ == '__main__':

    parser = arfunc.CreateParser(Title)

    # Странный финт
    # Первый и два последних аргумента (__name, __log) не нужны
    # Сделан потому, что при запуске roslaunch добавляются агрументы "__name:=...", и "__log:=..."
    arglist = []
    for e in sys.argv:
        if e.find("__name:=")==0: continue
        if e.find("__log:=")==0: continue
        arglist.append(e)
    namespace = parser.parse_args(arglist[1:])
    if namespace.nowindow is None: namespace.nowindow = False
    if namespace.nooutput is None: namespace.nooutput = False
    if namespace.cfg is None: namespace.cfg = os.path.dirname(os.path.realpath(__file__))+"/cfg/camera"

    arfunc.TMR = namespace.tmr # Инерционность сенсора, такты
    arfunc.CAMERA_ANGLE = namespace.camang
    arfunc.NUM_PIXELS = namespace.width
    arfunc.markerLength = namespace.msize
    RosFreq = namespace.rfreq

    print(namespace)

    arfunc.RLOC = [[0,0,0]]*arfunc.CAMERA_ANGLE
    main(namespace.input, namespace.cfg, namespace.topic, not namespace.nowindow, not namespace.nooutput)
