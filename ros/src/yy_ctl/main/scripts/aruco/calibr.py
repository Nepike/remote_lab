#!/usr/bin/python3
# -*- coding: UTF-8 -*-
"""
  V 1.1
  12.06.2021
  LP 01.08.2021
  Утилита создания калибровочных матриц.
  На входе принимает изображения (шахматной доски 8х8), на выходе формирует 2 файла.
  Параметры:
  -r 8 - количество клеток по вертикали
  -c 8 - количество клеток по горизонтали
  -o ./cfg/outp - имя выходного каталога и файла
  --img "./images/calibrate/*.png" - - имя входного каталога и маска файлов
"""

import cv2
import numpy as np
import sys, os
import glob
import argparse

from yyctl import arfunc

Title = 'Calibrate Utility 1.1'

def ExecuteImages(imagesdir, NCOLS, NROWS, outputfile):
    # NCOLS, NROWS - размеры шахматной доски

    CHECKERBOARD = (NCOLS-1, NROWS-1)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Создание вектора для хранения векторов трехмерных точек для каждого изображения шахматной доски
    objpoints = []
    # Создание вектора для хранения векторов 2D точек для каждого изображения шахматной доски
    imgpoints = [] 

    # Определение мировых координат для 3D точек
    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    prev_img_shape = None

    # Извлечение пути отдельного изображения, хранящегося в данном каталоге
    images = glob.glob(imagesdir)
    for fname in images:
        img = cv2.imread(fname)

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Найти углы шахматной доски
        # Если на изображении найдено нужное количество углов, тогда ret = true
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
        print(fname, ret)
        """
        Если желаемый номер угла обнаружен,
        уточняем координаты пикселей и отображаем
        их на изображениях шахматной доски
        """
        if ret == True:
            objpoints.append(objp)
            # уточнение координат пикселей для заданных 2d точек.
            corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
            
            imgpoints.append(corners2)

            # Нарисовать и отобразить углы
            img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        
        cv2.imshow('image', img)
        cv2.waitKey(0)

    cv2.destroyAllWindows()

    h, w = img.shape[:2]
    print(f"h = {h}, w = {w}")

    """
    Выполнение калибровки камеры.
    Передача значения известных трехмерных точек (объектов)
    и соответствующие пиксельные координаты
    обнаруженные углы (imgpoints)
    """
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    f = open(outputfile,"w")

    print("\n# Camera matrix:", file=f)
    arfunc.PrintVect("camera_matrix = array(", ")\n", mtx, f)

    print("\n# dist:", file=f)
    arfunc.PrintVect("dist = array(", ")\n", dist, f)

    print("\n# rvecs:", file=f)
    arfunc.PrintVect("rvecs = ", "\n", rvecs, f)

    print("\n# tvecs:", file=f)
    arfunc.PrintVect("tvecs = ", "\n", tvecs, f)

    f.close()

    arfunc.saveCoefficients(mtx, dist, outputfile+".yaml")

#
#
#
 
if __name__ == '__main__':
    parser = argparse.ArgumentParser(prog = '',  description = Title,  epilog = '')

    parser.add_argument ('-r', '--rows', default=None, required=True, type=int, metavar = 'row number',
                         help = 'chessboard row number')
    parser.add_argument ('-c', '--cols', default=None, required=True, type=int, metavar = 'columns number',
                         help = 'chessboard columns number')

    parser.add_argument ('-o', '--out', default=None, required=True, metavar = 'resfile',
                         help = 'output resfile')
    parser.add_argument ('-i', '--img', default=None, required=True, metavar = 'jpeg-dir',
                         help = 'input jpeg-dir')

    namespace = parser.parse_args(sys.argv[1:])

    ExecuteImages(namespace.img, namespace.cols, namespace.rows, namespace.out)
