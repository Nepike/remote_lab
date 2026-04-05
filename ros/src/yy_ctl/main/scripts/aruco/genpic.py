#!/usr/bin/python3
# -*- coding: UTF-8 -*-
"""
  Генератор ArUco-маркеров
  V 1.2
  12.06.2021
  LP 10.03.2025
"""
import cv2 as cv
import numpy as np
import sys, argparse

Title = 'ArUco Pics Generator 1.2'

# Загрузить предопределенный словарь
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
adict = cv.aruco.DICT_5X5_50

dictionary = cv.aruco.Dictionary_get(adict)

ID_LIST = [i for i in range(50)]

def main(outdir, picsize, prefix):
    # Сгенерировать маркер
    markerImage = np.zeros((picsize, picsize), dtype=np.uint8)
    for id in ID_LIST:
        markerImage = cv.aruco.drawMarker(dictionary, id, picsize, markerImage, 1);

        border_width = int(picsize / 10)
        markerImage = cv.copyMakeBorder(markerImage, border_width, border_width, border_width, border_width,     borderType=cv.BORDER_CONSTANT, value=255)        

        fname = outdir+"/"+prefix+str(id)+".png"
        print("-->", fname)
        cv.imwrite(fname, markerImage);

if __name__ == '__main__':
    parser = argparse.ArgumentParser(prog = '',  description = Title,  epilog = '')

    parser.add_argument ('-o', '--out', default=None, required=True, metavar = 'resdir',
                         help = 'output dir')
    parser.add_argument ('-size', '--size', type=int, default=800, required=False, metavar = 'picsize',
                         help = 'picture resolution, default is 800')
    parser.add_argument ('-p', '--prefix', type=str, default='', required=False, metavar = 'prefix',
                         help = 'picture filename prefix, default is ""')

    namespace = parser.parse_args(sys.argv[1:])

    main(namespace.out, namespace.size, namespace.prefix)
