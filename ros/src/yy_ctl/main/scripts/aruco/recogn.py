#!/usr/bin/python3
# -*- coding: utf-8 -*- 
'''
  Демонстрация распознавания ArUco-маркеров
  12.06.2021
  LP 30.01.2022

'''

import cv2 as cv
import numpy as np
import argparse, sys, os
from numpy import array

from yyctl import arfunc

adict = cv.aruco.DICT_4X4_50

markerLength = 0.05 # длина стороны маркера, м

def main(device, configfile):
    # Загрузить словарь, который использовался для создания маркеров.
    dictionary = cv.aruco.Dictionary_get(adict)

    # Инициализировать параметры детектора, используя значения по умолчанию
    parameters =  cv.aruco.DetectorParameters_create()

    # Обнаружение маркеров на изображении
    cv.namedWindow("WebCam")
    device = arfunc.GetDevN(device)
    vc = cv.VideoCapture(device) # Цепляем видеопоток с вебкамеры (устройство 0,1,...)

    if vc.isOpened(): # Пробуем получить первый кадр
        rval, frame = vc.read() # rval - флаг успеха, frame - сам кадр 
    else:
        rval = False

    camera_matrix, dist_coeff = arfunc.loadCoefficients(configfile+".yaml")
    if (camera_matrix is None) or (dist_coeff is None):
        print("*** error loading config file "+configfile+".yaml")
        sys.exit(1)

    print('camera_matrix =', camera_matrix)

    while rval: # пока с кадрами всё в порядке
        rval, frame = vc.read() # получаем новый кадр

        if not (frame is None):
            markerCorners, markerIds, rejectedCandidates = cv.aruco.detectMarkers(frame, dictionary, parameters=parameters)
            rvec, tvec, _ = cv.aruco.estimatePoseSingleMarkers(markerCorners, markerLength, camera_matrix, dist_coeff)

            if not (markerIds is None):
                cv.aruco.drawDetectedMarkers(frame, markerCorners)  # Draw A square around the markers
                print(markerIds, markerCorners)
                print(rvec, tvec)

            cv.imshow("WebCam", frame) # Отображаем окно и выводим на него frame

        key = cv.waitKey(10) # Ждём нажатия клавиши
        if key == 27: # если нажата клавиша ESC, выходим
            break

    cv.destroyWindow("WebCam") # убиваем окно 
    vc.release() # закрываем видеопоток

#
#
#
 
if __name__ == '__main__':
    parser = argparse.ArgumentParser(prog = '',  description = 'title',  epilog = '')

    parser.add_argument ('-d', '--device', default=None, required=True, metavar = 'device', 
                        help='device: 0,1... or /dev/video0...')

    parser.add_argument ('-c', '--cfg', default=None, required=True, metavar = 'configfilename')

    namespace = parser.parse_args(sys.argv[1:])

    main(namespace.device, namespace.cfg)
