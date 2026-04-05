#!/bin/bash

#IMG_TOPIC=/usb_cam/image_raw 
IMG_TOPIC=/group_mobot1/mobot1/camera1/image_raw

./aruco_server.py --input $IMG_TOPIC --cfg cfg/camera --topic superlocator --tmr 5

