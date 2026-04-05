#!/usr/bin/python3
# -*- coding: utf-8 -*- 
"""
  V 1.1
  12.06.2021
  LP 01.08.2021
  Утилита формирования снимков с камеры для ее калибровки.
  В качестве изображения используется шахматная доска 8х8.
  Параметры:
  device - устройство (камера)
  outdir - директория, в которую будут сохраняться снимки с камеры
  cam_col - количество пикселей по горизонтали
  cam_row - количество пикселей по вертикали
  
  Тип файлов устанавливается здесь же (в этой версии - png).
"""

import cv2 as cv
import numpy as np
import argparse, sys, os

from yyctl import arfunc

Title = 'VCap Utility 1.1'

def main(device, outdir, cam_col, cam_row):

    cv.namedWindow("WebCam")

    device = arfunc.GetDevN(device)

    cap = cv.VideoCapture(device) # Цепляем видеопоток с вебкамеры (устройство №0)

    arfunc.make_res(cap, (cam_col, cam_row))

    if cap.isOpened(): # Пробуем получить первый кадр
        rval, frame = cap.read() # rval - флаг успеха, frame - сам кадр 
    else:
        rval = False

    n = 0

    while rval: # пока с кадрами всё в порядке
        rval, frame = cap.read() # получаем новый кадр
        cv.imshow("WebCam", frame) # Отображаем окно и выводим на него frame

        key = cv.waitKey(10) # Ждём нажатия клавиши
        if key == 27: # если нажата клавиша ESC, выходим
            break
        if key in [ord(' '),ord('w'), ord('W')]: # Пробел. Делаем снимок
            n += 1
            fname = outdir+"/pic"+str(n)+".png"
            print('save', fname)
            cv.imwrite(fname, frame);

    cv.destroyWindow("WebCam") # убиваем окно
    cap.release() # закрываем видеопоток

#
#
#
 
if __name__ == '__main__':
    parser = argparse.ArgumentParser(prog = '', description = Title, epilog = '')

    parser.add_argument ('-d', '--device', default=None, required=True, metavar = 'device', 
                        help='device: 0,1... or /dev/video0...')

    parser.add_argument ('-o', '--out', default=None, required=True, metavar = 'dir', help = 'output dir')
    parser.add_argument ('-r', '--rows', default=480, required=False, type=int, metavar = 'row number',
                         help = 'camera row number, default = 480')
    parser.add_argument ('-c', '--cols', default=640, required=False, type=int, metavar = 'columns number',
                         help = 'camera columns number, default = 640')

    namespace = parser.parse_args(sys.argv[1:])

    main(namespace.device, namespace.out, namespace.cols, namespace.rows)
