#!/usr/bin/env python3
# coding: utf-8
"""
  MOBOT_DU-KVORUM2 Bridge
  Программа согласования топиков
  
  01.03.2024, 17.11.2024

  Version 2.4

  LP 15.01.2025

  Запускается для конкретного робота rid
  Значения датчиков в метрах, за исключением
  -- ArUco-сервер (формирует нормальный модельный поток)
  -- Servolocator, см. Переводится в метры.
  Их надо переводить в модельные значения
"""

import os, sys, time, random, math
import argparse
import roslib, rospy
import configparser

from kvorum2 import gdic
from gmodctl import quat_euler
from gmodctl import mbridge

# Топики ROS
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan

from msg_kvorum2.msg import action
from msg_kvorum2.msg import sdata, sensors

from geometry_msgs.msg import Twist
from msg_yy.msg import servolocator

###################################################################

Title = "MOBOT_DU-KVORUM2 Bridge 2.4"

Pub_sensors = None
Pub_vel = None

_RID_ = None

SERVER_ADDR = 100

show_log = False

###################################################################

def log(s):
    if show_log:
        print(s)

###################################################################
#
# Свалим все управляющие параметры в одну кучу
#
class TCtlParams:
    def __init__(self):
        self.cmd = 0
        self.vlinear = 0
        self.vangular = 0
        self.angles = [0,0,0]
        self.buttons = [0,0,0]

###################################################################

CtlP = TCtlParams()

def RIDSUBST(s):
  v = s.replace("{RID}", str(_RID_))
  return v

################################################################################
#
# Отправка команд роботу
# (Управление скоростью)
#
################################################################################
def mobot_SendVelCmd(pub):
    velmsg = Twist()
    velmsg.linear.x = CtlP.vlinear
    velmsg.angular.z = CtlP.vangular
    pub.publish(velmsg)

################################################################################
#
# Сообщения от Kvorum2
#
################################################################################
def rsaction_callback(cmdmsg):
    global Pub_sensors, Pub_vel

    cmd = cmdmsg.action
    aid = cmdmsg.agent_id
    if aid!=_RID_: return # Это не мне
    slist = cmdmsg.slist

    arg1 = cmdmsg.arg1
    arg2 = cmdmsg.arg2
    arg3 = cmdmsg.arg3

    #
    # Запрос данных
    #
    if(cmd == gdic.CMD_GET_SENS):
        msg_sens = sensors()
        msg_sens.data = []
        msg_sens.src = SERVER_ADDR
        msg_sens.dest = aid #gdic.MY_ADDR
        msg_sens.tm = int(time.time())
        # slist содержит имена запрашиваемых сенсоров
        for sname in slist:
            sname=sname.upper()
            # С некоторыми именами разбираемся особо
            if sname=='POSITION':
                sensdata = [0, 0, mbridge.PrimarySensors['compass']]
            elif sname=='LOCATOR':
                # Возвращает значения в метрах. Переводим в единицы Kvorum
                sensdata = [int(r/mbridge.cellsize) if r!=math.inf else 0  for r in mbridge.PrimarySensors['lidar']]
            elif sname=='SUPERLOCATOR':
                # Возвращает расстояние в сантиметрах. Переводим в единицы Kvorum
                sensdata = gdic.SuperVector2Vector(mbridge.PrimarySensors['arucolocator'], 0.01/mbridge.cellsize)
            elif sname.find("DET_IR_BEACON_")==0:
                sensdata = mbridge.PrimarySensors[sname]
            else:
                if sname in KVSensName2MobotSensName.keys(): # Дальномеры
                    mobot_sens_name = KVSensName2MobotSensName[sname]
                    # Возвращает значения в метрах. Переводим в единицы Kvorum
                    sensdata = int(mbridge.PrimarySensors[mobot_sens_name]/mbridge.cellsize)
                else:
                    continue
            if type(sensdata)!=list: sensdata = [sensdata]

            sval = sdata()
            sval.args = sname
            sval.data = sensdata
            msg_sens.data.append(sval)
            log(f"-- {sname}: sval.data={sval.data}")
        Pub_sensors.publish(msg_sens)
        return
    #
    # Движение
    #
    elif(cmd == gdic.CMD_FWD):
        #print("-- FWD")
        CtlP.vlinear, CtlP.vangular = mbridge.cspeed, 0
    elif(cmd == gdic.CMD_BACK):
        #print("-- BACK")
        CtlP.vlinear, CtlP.vangular = -mbridge.cspeed, 0
    elif(cmd == gdic.CMD_SET_CURR_2W_SPEED):
        #A.SetSidesSpeed(arg1, arg2)
        pass
    elif(cmd == gdic.CMD_LEFT or cmd == gdic.CMD_FAST_LEFT):
        #print("-- LEFT")
        CtlP.vlinear, CtlP.vangular = mbridge.rotvlin, math.radians(mbridge.rotvang)
    elif(cmd == gdic.CMD_RIGHT or cmd == gdic.CMD_FAST_RIGHT):
        #print("-- RIGHT")
        CtlP.vlinear, CtlP.vangular = mbridge.rotvlin, math.radians(-mbridge.rotvang)
    elif(cmd == gdic.CMD_STOP):
        CtlP.vlinear, CtlP.vangular = 0, 0
        #print("-- STOP")
    elif(cmd == gdic.CMD_SET_CURR_V_SPEED):
        CtlP.vlinear, CtlP.vangular = arg1, math.radians(arg2)
    elif(cmd == gdic.CMD_SET_NORM_V_SPEED):
        mbridge.cspeed = arg1
        #mbridge.rotvang = math.radians(arg2)
    # Публикуем
    mobot_SendVelCmd(Pub_vel)

################################################################################
#
# Сообщения от MOBOT_DU
# Все складируется в mbridge.PrimarySensors
#
################################################################################
# Словарь для имен Mobot-Kvorum2
KVSensName2MobotSensName = {
    "USF_GND_FWD":           'usonic_fwd_center',
    "USF_GND_FWD_LEFT" :     'usonic_fwd_left',
    "USF_GND_FWD_RIGHT":     'usonic_fwd_right',
    "USF_GND_SIDE_LEFT_FWD": 'usonic_side_left_fwd',
    "USF_GND_SIDE_RIGHT_FWD":'usonic_side_right_fwd',
    "USF_GND_SIDE_LEFT_BCK": 'usonic_side_left_bck',
    "USF_GND_SIDE_RIGHT_BCK":'usonic_side_right_bck'}

################################################################################
#
# MAIN
#
################################################################################
def data_lce_cb(msg):
    for d in msg.data:
        aid = int(d.args)
        if aid!=_RID_: continue # Это не мне
        mbridge.PrimarySensors["DET_IR_BEACON_FWD"] = d.data[0]
        mbridge.PrimarySensors["DET_IR_BEACON_BACK"] = d.data[1]
        mbridge.PrimarySensors["DET_IR_BEACON_LEFT"] = d.data[2]
        mbridge.PrimarySensors["DET_IR_BEACON_RIGHT"] = d.data[3]

def main(configfile, rid, log):
    global Pub_sensors, Pub_vel
    global _RID_
    global show_log

    show_log = log

    _RID_ = rid
    gdic.MY_ADDR = rid

    config = configparser.ConfigParser()
    config.read(configfile)

    ############################################################################
    # Инициалиизация ROS
    ############################################################################
    rospy.init_node("mobot_du_kvorum_bridge")

    #
    # Входные топики
    #
    # Источник -- KVORUM:
    rospy.Subscriber(gdic.TOPIC_NAME_COMMAND, action, rsaction_callback, queue_size = 1)
    #

    # Источник -- сервер ArUco-маркеров:
    rospy.Subscriber(RIDSUBST(config['EXTTOPICS']['arucolocator']), sensors, mbridge.data_superlocator_cb, queue_size = 1)
    # Источник -- MOBOT_DU
    rospy.Subscriber(RIDSUBST(config['INPTOPICS']['imutopic']), Imu, mbridge.mobot_imu_cb)
    rospy.Subscriber(RIDSUBST(config['INPTOPICS']['lidar']), LaserScan, mbridge.mobot_lidar_cb)
    rospy.Subscriber(RIDSUBST(config['SERVOLOCATOR']['locator']), servolocator, mbridge.mobot_servolocator_cb)

    # Источник -- эмулятор локальной сязи
    rospy.Subscriber(RIDSUBST(config['LCERC5']['output_topic']), sensors, data_lce_cb, queue_size = 1)

    for r in config.items('RANGETOPICS'):
        rospy.Subscriber(RIDSUBST(r[1]), Range, mbridge.mobot_rf_cb)
    #
    # Выходные топики
    #
    # Получатель -- MOBOT_DU
    Pub_vel = rospy.Publisher(RIDSUBST(config['OUTTOPICS']['veltopic']), Twist, queue_size = 1)

    # Получатель -- KVORUM
    Pub_sensors = rospy.Publisher(gdic.TOPIC_NAME_SENSORS, sensors, queue_size = 1)

    try:
        rospy.get_param_names()
    except ROSException:
        print("could not get param name")

    mbridge.Init()

    #####################################################
    # Основной цикл
    #####################################################
    print("Start main loop")

    rate = rospy.Rate(10) # 10 Hz

    while not rospy.is_shutdown():
        # Будем с частотой 10 Гц отправлять команды в любом случае, даже повторяя старые
        # Получатель -- MOBOT_DU. Управление скоростью
        #mobot_SendVelCmd(Pub_vel)

        rate.sleep()

    gdic.terminate_program()

################################################################################
#
################################################################################
if __name__ == '__main__':

    print(Title)
    parser = argparse.ArgumentParser(prog = '', description = Title, epilog = '')
    parser.add_argument('--cfg', nargs='?', default=None, required=True, metavar = 'filename', help = 'topics description filename')
    parser.add_argument('--rid', nargs='?', type=int, default=1, required=True, metavar = 'robot_id', help = 'robot id (id=1,2...)')
    parser.add_argument('--log', const=True, required=False, metavar='', action='store_const', help = 'log sensors info')

    # Странный финт
    # Первый и два последних аргумента (__name, __log) не нужны
    # Сделан потому, что при запуске roslaunch добавляются агрументы "__name:=...", и "__log:=..."
    arglist = []
    for e in sys.argv:
        if e.find("__name:=")==0: continue
        if e.find("__log:=")==0: continue
        arglist.append(e)

    namespace = parser.parse_args(arglist[1:])
    print(namespace)

    main(namespace.cfg, namespace.rid, namespace.log)
