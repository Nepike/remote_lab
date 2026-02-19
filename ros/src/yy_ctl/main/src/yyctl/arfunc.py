#!/usr/bin/python3
# -*- coding: UTF-8 -*-
"""
  Библиотека полезных функций для aruco-сервера
  V 2.2
  
  01.06.2021, 30.01.2022, 30.07.2023
  LP 19.03.2025
  
"""

import cv2 as cv
import numpy as np
import sys, os
import glob
import argparse

TMR = None            # 10 Инерционность сенсора, такты
CAMERA_ANGLE = None   # 60 Угол обзора камеры, град
NUM_PIXELS = None     # 640 Количество пикселей по горизонтали
markerLength = None   # 0.03 Длина стороны маркера, м

# Локатор
# [[dist, id, tmr]]
#    dist -- расстояние в сантиметрах,
#    id   -- код ArUco-маркера,
#    tmr  -- время релаксации (количество тактов)
RLOC = None

#
#
#
def saveCoefficients(mtx, dist, path):
    # Save the camera matrix and the distortion coefficients to given path/file
    cv_file = cv.FileStorage(path, cv.FILE_STORAGE_WRITE)
    cv_file.write("camera_matrix", mtx)
    cv_file.write("dist_coeff", dist)
    # note you *release* you don't close() a FileStorage object
    cv_file.release()

#
#
#
def loadCoefficients(path):
    # Loads camera matrix and distortion coefficients.
    # FILE_STORAGE_READ
    cv_file = cv.FileStorage(path, cv.FILE_STORAGE_READ)

    # note we also have to specify the type to retrieve other wise we only get a
    # FileNode object back instead of a matrix
    camera_matrix = cv_file.getNode("camera_matrix").mat()
    dist_matrix = cv_file.getNode("dist_coeff").mat()

    # Debug: output the values
    # print("camera_matrix : ", camera_matrix.tolist())
    # print("dist_matrix : ", dist_matrix.tolist())

    cv_file.release()
    return [camera_matrix, dist_matrix]
#
#
#
def PrintVect(prefix, suffix, V, f):
    print(prefix, end = '', file=f)

    if type(V)!=np.ndarray:
        print(V, file=f)
        return

    print('[', end = '', file=f)

    for i in range(len(V)-1):
        if type(V[i])==np.ndarray:
            PrintVect('', '', V[i], f)
        else:
            print(V[i], end = '', file=f)
        print(', ', end = '', file=f)

    last = -1
    if type(V[last])==np.ndarray:
        PrintVect('', '', V[last], f)
    else:
        print(V[last], end = '', file=f)

    print(']' + suffix, end = '', file=f)

#
# resolution: (1920,2080) (1280, 720) (640, 480)
#
def make_res(cap, resolution):
    global image_width
    global image_height
    image_width, image_height = resolution[0], resolution[1]
    cap.set(3, image_width)
    cap.set(4, image_height)

#
#
#
def GetDevN(s):
  try:
      nd = int(s)
  except:
      nd = s
  return nd

#
# camCorn - угол обзора (60)
# numx - количество пикселей по горизонтали
# mIds - массив идентификаторов маркеров
# mCorn - массив координат углов маркеров
# tVec - вектор перемещения
#
def Plocator(camAngle, numx, mIds, mCorn, tVec):
    loc = [[0, 0]]*camAngle
    coef = numx/camAngle # пикселей на один градус
    for i in range(len(mIds)):
        xmin = int(min(mCorn[i][0][0][0], mCorn[i][0][1][0], mCorn[i][0][2][0], mCorn[i][0][3][0])/coef) # left coordinate
        xmax = int(max(mCorn[i][0][0][0], mCorn[i][0][1][0], mCorn[i][0][2][0], mCorn[i][0][3][0])/coef) # right coordinate
        # markers.append([mIds[0][i], int(xmin/coef), int(xmax/coef), tVec[i][0][2]]) # id, left, right, distance
        dist = int(tVec[i][0][2] * 100.0) # Возвращаем дистанцию в см
        for n in range(xmin, min(xmax+1, camAngle)): 
            loc[n] = [ dist, mIds[i][0] ]
    return loc

######################################################################


#
# camCorn - угол обзора (60)
# numx - количество пикселей по горизонтали
# mIds - массив идентификаторов маркеров
# mCorn - массив координат углов маркеров
# tVec - вектор перемещения
#
def Plocator2(camAngle, numx, mIds, mCorn, tVec):
    coef = numx/camAngle # пикселей на один градус
    for i in range(len(mIds)):
        xmin = int(min(mCorn[i][0][0][0], mCorn[i][0][1][0], mCorn[i][0][2][0], mCorn[i][0][3][0])/coef) # left coordinate
        xmax = int(max(mCorn[i][0][0][0], mCorn[i][0][1][0], mCorn[i][0][2][0], mCorn[i][0][3][0])/coef) # right coordinate
        # markers.append([mIds[0][i], int(xmin/coef), int(xmax/coef), tVec[i][0][2]]) # id, left, right, distance
        # sic Параноидальные проверки
        dist = int(tVec[i][0][2] * 100.0) # Возвращаем дистанцию в см
        mids = mIds[i][0]
        if (dist is None) or (dist<=0) or (dist>1000): dist, mids = 0, 0
        if (mids is None) or (mids<0) or (mids>250): dist, mids = 0, 0
        for n in range(xmin, min(xmax+1, camAngle)):
            RLOC[n] = [ dist, mids, TMR ]
    return

#
#
#
def ShowLocator(loc):
    max_dist = max(loc, key=lambda i : i[0])
    LOCNUM = len(loc)
    print(f"MD = {max_dist} ({LOCNUM})")
    print(''.rjust(LOCNUM,'_'))
    NSTEP = 10
    MAXVAL = 100
    for n in range(NSTEP):
        crval = (NSTEP-n-1)*(MAXVAL // NSTEP)
        s = ''
        for i in range(LOCNUM):
            lval = min(loc[i][0], MAXVAL)
            if (lval>crval): s = s + "{}".format(int(loc[i][1]))
            else: s = s + "."
        print(s)
    print(''.rjust(LOCNUM,'_'))

#
#
#
def CreateParser(title):
    parser = argparse.ArgumentParser(prog = '', description = title, epilog = '')

    parser.add_argument ('--input', default=None, required=True, metavar = 'inptopic', 
                        help='input topic name, example: /usb_cam/image_raw')


    parser.add_argument ('--topic', default=None, required=True, metavar = 'outtopic',
                         help = 'output topic name')

    parser.add_argument ('--cfg', default=None, required=False, metavar = 'config',
                         help = 'config file, default is "cfg/camera"')

    parser.add_argument ('--nowindow', const=True, required=False, metavar='', action='store_const',
                         help = 'use only console, without image window')

    parser.add_argument ('--nooutput', const=True, required=False, metavar='', action='store_const',
                         help = 'don\'t output')

    # Инерционность сенсора, такты
    parser.add_argument('--tmr', type=int, default=100, required=False, metavar='', help='TMR value (default is 100)')

    # Угол обзора камеры, град
    parser.add_argument('--camang', type=int, default=60, required=False, metavar='', help='Camera angle, grad (default is 60)')

    # Количество пикселей по горизонтали
    parser.add_argument('--width', type=int, default=640, required=False, metavar='', help='Image width (default is 640)')

    # Длина стороны маркера, м
    parser.add_argument('--msize', type=float, default=0.03, required=False, metavar='', help='ArUco Marker Size, m (default is 0.03)')

    # Частота ROS, Гц
    parser.add_argument('--rfreq', type=int, default=10, required=False, metavar='', help='Ros Rate, Hz (default is 10)')

    # id робота (по умолчанию=1)
    parser.add_argument('--rid', nargs='?', type=int, default=1, required=False, metavar = 'robot_id', help = 'robot id (id=1,2...)')

    return parser
