#!/usr/bin/env python3
# coding: utf-8

import sys

import roslib
import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

global bridge

def img_callback(data):
    global bridge
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except  e :
        print(e)

    cv2.imshow("Camera 0", cv_image)
    cv2.waitKey(3)

def main(args) :
    global bridge
    rospy.init_node('test_video')

    bridge = CvBridge()
    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()

if __name__ == '__main__' :
    main(sys.argv)
