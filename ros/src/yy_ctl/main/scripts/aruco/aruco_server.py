#!/usr/bin/env python3
# coding: utf-8
"""
  ArUco_server (Kvorum2)
  ROS Version

  V 2.4
  12.06.2021, 30.01.2022, 15.05.2023, 30.07.2023, 11.03.2024, 06.12.2024
  LP 19.03.2025

  Публикует в msg_yy.msg/ans: "superlocator"
  Возвращает супервектор (расстояние, код)
  расстояние - в сантиметрах.
  Настойка расстояния осуществляется путем изменения аргумента msize - размера ArUco-маркера в м.
"""
import cv2 as cv
import numpy as np
import argparse, os, sys, time
from numpy import array

import roslib, rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from yyctl import arfunc
from kvorum2 import gdic
from msg_kvorum2.msg import sdata, sensors

Title = 'ArUco Server 2.4'

SERVER_ADDR = 100
'''
cv.aruco.DICT_4X4_50
cv.aruco.DICT_4X4_100
cv.aruco.DICT_4X4_250
cv.aruco.DICT_4X4_1000
cv.aruco.DICT_5X5_50
cv.aruco.DICT_5X5_100
cv.aruco.DICT_5X5_250
cv.aruco.DICT_5X5_1000
cv.aruco.DICT_6X6_50
cv.aruco.DICT_6X6_100
cv.aruco.DICT_6X6_250
'''
#adict = cv.aruco.DICT_4X4_50
adict = cv.aruco.DICT_5X5_50 # В модели распознается хорошо

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
# Отправка сообщения (ans.msg)
#
def SendSuperLocator(pub, superlocator):
    msg_sens = sensors()
    msg_sens.data = []
    msg_sens.src = SERVER_ADDR
    msg_sens.dest = gdic.MY_ADDR
    msg_sens.tm = int(time.time())

    # Переворачиваем
    superlocator.reverse()
    sldata = gdic.SuperVector2Vector(superlocator, 1)

    sval = sdata()
    sval.args = "SUPERLOCATOR"
    sval.data = sldata
    msg_sens.data.append(sval)

    pub.publish(msg_sens)

######################################################################

RosFreq = None

def main(inptopicname, configfile, outtopicname, show_window, print_result):
    global bridge, cv_image
    global RosFreq

    # Загрузить словарь, который использовался для создания маркеров
    try:
        dictionary = cv.aruco.Dictionary_get(adict)
    except:
        dictionary = cv.aruco.getPredefinedDictionary(adict)

    # Инициализировать параметры детектора, используя значения по умолчанию
    try:
        parameters = cv.aruco.DetectorParameters_create()
    except:
        parameters =  cv.aruco.DetectorParameters()

    # Initialize the CvBridge class
    bridge = CvBridge()

    rospy.init_node('aruco_server')

    # Входной топик
    rospy.Subscriber(inptopicname, Image, img_callback)

    # Выходной топик
    pub_superlocator = rospy.Publisher(outtopicname, sensors, queue_size = 1)

    camera_matrix, dist_coeff = arfunc.loadCoefficients(configfile+".yaml")
    if (camera_matrix is None) or (dist_coeff is None):
        print("*** error loading config file "+configfile+".yaml")
        sys.exit(1)

    LOC = [[0,0]]*arfunc.CAMERA_ANGLE

    rate = rospy.Rate(RosFreq)
    while not rospy.is_shutdown():

        for i in range(len(arfunc.RLOC)):
            if arfunc.RLOC[i][2]>0: arfunc.RLOC[i][2] -=1
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

        # Публикуем
        # sic Параноидальные проверки
        for i in range(len(arfunc.RLOC)):
            dist, mids = arfunc.RLOC[i][0], arfunc.RLOC[i][1]
            if (dist is None) or (dist<=0) or (dist>1000): dist, mids = 0, 0
            if (mids is None) or (mids<0) or (mids>250): dist, mids = 0, 0
            LOC[i] = [dist, mids]
        SendSuperLocator(pub_superlocator, LOC)

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
    gdic.MY_ADDR = namespace.rid

    print(namespace)

    arfunc.RLOC = [[0,0,0]]*arfunc.CAMERA_ANGLE
    main(namespace.input, namespace.cfg, namespace.topic, not namespace.nowindow, not namespace.nooutput)
