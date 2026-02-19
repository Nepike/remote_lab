#!/usr/bin/env python3
# coding: utf-8
"""

  Управление локатором на сервоприводе
  Все данные - в сантиметрах для совместимости с железом
  17.03.2024

  Version 1.04

  LP 28.06.2024

"""
import os, sys, time, random, math
import argparse
import roslib, rospy
import configparser

# Топики ROS
from sensor_msgs.msg import Range
from std_msgs.msg import Float64 as DataCtl

from msg_yy.msg import servolocator

###################################################################

Title = "ServoLocator 1.04"

Pub_ans = None
Pub_vel = None

_RID_ = None

class TServoLocator:
    def __init__(self):
        self.A1 = -90
        self.A2 = 90
        self.A = int((self.A1+self.A2)/2)
        self.rfval = 0
        self.delta = 4
        self.cdir = 1
        self.range_min = 0
        self.range_max = 0

        self.dim = int(self.A2-self.A1+1)
        self.locator = [0]*self.dim

    def getangle(self):
        self.A += self.cdir*self.delta
        if self.A>self.A2:
            self.A=self.A2
            self.cdir = -1
        if self.A<self.A1:
            self.A=self.A1
            self.cdir = 1
        return math.radians(self.A)

    # Для совместимости с железом будем публиковать данные в сантиметрах, а не в метрах
    def accept(self, msg):
        idx = int(self.A-self.A1)
        self.rfval = 100*int(msg.range*1000)/1000.0
        self.range_min = 100*int(msg.min_range*1000)/1000.0
        self.range_max = 100*int(msg.max_range*1000)/1000.0

        self.rfval = int(self.rfval)
        self.range_min = int(self.range_min)
        self.range_max = int(self.range_max)

        if self.rfval<=self.range_min or self.rfval>=self.range_max:
            self.rfval = 0
        # Заполняем значения между шагами
        for i in range(self.delta):
            if idx+i>=0 and idx+i<self.dim:
                self.locator[idx+i] = self.rfval

RF = TServoLocator()

###################################################################
def pmap(x, x_min, x_max, out_min, out_max):
    if (x>x_max): x = x_max
    if (x<x_min): x = x_min
    r = (x - x_min) * (out_max - out_min) / (x_max - x_min) + out_min
    return int(r)

################################################################################
#
# Отправка команд роботу
#
################################################################################

# Управление скоростью
def mobot_SendVelCmd(pub):
    velmsg = Twist()
    velmsg.linear.x = CtlP.vlinear
    velmsg.angular.z = CtlP.vangular
    pub.publish(velmsg)

################################################################################
#
# Сообщения от MOBOT_DU
# Все складируется в Sensors
#
################################################################################
def mobot_rf_cb(msg):
    RF.accept(msg)
    return

################################################################################

def RIDSUBST(s): return s.replace("{RID}", str(_RID_))

################################################################################
#
# MAIN
#
################################################################################

def main(configfile, rid):
    global _RID_

    _RID_ = rid

    config = configparser.ConfigParser()
    config.read(configfile)

    ############################################################################
    # Инициалиизация ROS
    ############################################################################
    rospy.init_node("servoctl")

    #
    # Входные топики
    #
    rospy.Subscriber(RIDSUBST(config['SERVOLOCATOR']['data']), Range, mobot_rf_cb, queue_size = 1)

    #
    # Выходные топики
    #
    Pub_ctl = rospy.Publisher(RIDSUBST(config['SERVOLOCATOR']['ctl']), DataCtl, queue_size = 1)

    Pub_locator = rospy.Publisher(RIDSUBST(config['SERVOLOCATOR']['locator']), servolocator, queue_size = 1)

    try:
        rospy.get_param_names()
    except ROSException:
        print("could not get param name")

    #####################################################
    # Основной цикл
    #####################################################
    print("Start main loop")

    rate = rospy.Rate(50) # 10 Hz

    msg = DataCtl()
    srvmsg = servolocator()
    srvmsg.A1 = RF.A1
    srvmsg.A2 = RF.A2

    while not rospy.is_shutdown():

        # Управление сервоприводом
        msg.data = RF.getangle()
        Pub_ctl.publish(msg)

        # Публикуем результаты (в сантиметрах)
        #
        srvmsg.A = int(RF.A)
        srvmsg.Val = RF.rfval
        srvmsg.data = RF.locator
        srvmsg.range_min = RF.range_min
        srvmsg.range_max = RF.range_max
        #print(srvmsg)
        Pub_locator.publish(srvmsg)

        rate.sleep()

    gdic.terminate_program()

################################################################################
#
################################################################################
if __name__ == '__main__':

    print(Title)
    parser = argparse.ArgumentParser(prog = '', description = Title, epilog = '')
    parser.add_argument('--cfg', nargs='?', default=None, required=True, metavar = 'filename', help = 'topics description filename')
    parser.add_argument('--rid', nargs='?', type=int, default=1, required=False, metavar = 'robot_id', help = 'robot id (id=1,2...)')

    # Странный финт
    # Первый и два последних аргумента (__name, __log) не нужны
    # Сделан потому, что при запуске roslaunch добавляются агрументы "__name:=...", и "__log:=..."
    arglist = []
    for e in sys.argv:
        if e.find("__name:=")==0: continue
        if e.find("__log:=")==0: continue
        arglist.append(e)

    namespace = parser.parse_args(arglist[1:])

    main(namespace.cfg, namespace.rid)
