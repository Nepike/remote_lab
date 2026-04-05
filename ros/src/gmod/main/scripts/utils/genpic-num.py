#!/usr/bin/python3
# -*- coding: UTF-8 -*-
"""
  Генератор картинок с числами
  V 1.1
  12.06.2021
  LP 29.02.2024
"""

import cv2 as cv
import numpy as np
import sys, argparse
from PIL import Image

Title = 'Num Pics Generator 1.1'

ID_LIST = [i for i in range(50)]

imSize = 250

def main(outdir, picsize, prefix):
    for id in ID_LIST:
        # Сгенерировать картинку
        numImage = Image.new('RGB', (imSize, imSize), (255, 255, 255))
        numImage = np.array(numImage)

        text = str(id)
        font = cv.FONT_HERSHEY_SIMPLEX
        font_size = 4
        color = (0, 0, 0) 
        thickness = 5

        (text_width, text_height) = cv.getTextSize(text, font, font_size, thickness)[0]
        pos = (int((imSize-text_width)/2), int((imSize-text_height)))

        cv.putText(numImage, text, pos, font, font_size, color, thickness)

        fname = outdir+"/"+prefix+str(id)+".png"
        print("-->", fname)

        cv.imwrite(fname, numImage);

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
