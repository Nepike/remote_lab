#!/usr/bin/env python3
# coding: utf-8
"""
  YARP13-KVORUM2 Bridge
  Программа согласования топиков
  Запускается для конкретного робота rid
  --rid id (robot`s id)
  По умолчанию id=1
  Если параметр указан явно, то к именам топиков робота добавляется окончание id

  12.06.2021, 22.08.2021, 22.03.2023, 02.07.2024, 23.12.2024

  Version 2.4

  LP 23.03.2025

"""
import os, sys, time, random, math
import argparse
import roslib, rospy

from kvorum2 import gdic

from yyctl import y13cmd

# Топики ROS
from msg_kvorum2.msg import action
from msg_kvorum2.msg import sdata, sensors

from msg_yy.msg import sens as yy_sens
from msg_yy.msg import cmd as yy_cmd
from geometry_msgs.msg import Twist

from msg_yy.msg import bt_data

###################################################################

Title = "YARP13-KVORUM2 Bridge 2.4"

compass_correction = None

cspeed = None # 0.25
rotvlin = None # 0.05
rotvang = None # 0.1
cellsize = None # Размер клетки Kvorum в метрах

_RID_ = 1         # id робота
SERVER_ADDR = 100 # id сервера

Pub_sensors = None

BtCompassData = None # Компас от Bluetooth

###################################################################
#
# Свалим все управляющие параметры в одну кучу
#
class TCtlParamsMove:
    def __init__(self):
        self.vlinear = 0
        self.vangular = 0

###################################################################

CtlPMove = TCtlParamsMove()
CtlPOth = yy_cmd()

############################################################

# Словарь показаний сенсоров робота YARP13
# Формируется на основе информации, полученной из топика робота
# Имя Yarp13 - (имя_Kvorum, значение)
Sensors = {
    'rf_center':         ["USF_GND_FWD", -1],
    'rf_left':           ["USF_GND_FWD_LEFT", -1],
    'rf_right':          ["USF_GND_FWD_RIGHT", -1],
    'rf_side_left_fwd':  ["USF_GND_SIDE_LEFT_FWD", -1],
    'rf_side_left_bck':  ["USF_GND_SIDE_LEFT_BCK", -1],
    'rf_side_right_fwd': ["USF_GND_SIDE_RIGHT_FWD", -1],
    'rf_side_right_bck': ["USF_GND_SIDE_RIGHT_BCK", -1],
    'rf_bck_center': [None, -1],

    'encoder_left':      [None, 0],
    'encoder_right':     [None, 0],

    'compass': [None, -1],
    'lidar':   [None, []],
    'arucolocator': [None, []],
    'aruco_camera_angle': [None, 0],

    'servolocator':[None, []],
    'irdata':[None, [0,0,0,0]]}

def robot_data_callback(msg):
    # Что-то делаем с компасом
    if not (BtCompassData is None):
        wcomp = int(BtCompassData)
    else:
        wcomp = msg.compass

    # То ли надо менять направление, то ли нет
    mcomp = (360 - wcomp) + compass_correction
    #mcomp = wcomp + compass_correction
    mcomp = mcomp % 360
    if mcomp<0: mcomp = 360 - mcomp

    # Компас
    Sensors['compass'][1] = mcomp
    # Передние дальномеры
    Sensors['rf_center'][1] = msg.rf_center
    Sensors['rf_left'][1] = msg.rf_left
    Sensors['rf_right'][1] = msg.rf_right

    # Боковые дальномеры
    Sensors['rf_side_left_fwd'][1] = msg.rf_side_left_fwd
    Sensors['rf_side_right_fwd'][1] = msg.rf_side_right_fwd
    Sensors['rf_side_left_bck'][1] = msg.rf_side_left_bck
    Sensors['rf_side_right_bck'][1] = msg.rf_side_right_bck
    Sensors['rf_bck_center'][1] = msg.rf_bck_center

    Sensors['encoder_left'][1] = msg.enc_left
    Sensors['encoder_right'][1] = msg.enc_right

    Sensors['irdata'][1] = msg.irdata
    return

#
# Сообщения от ArUco-сервера
# Возвращает расстояние в сантиметрах
#
def data_superlocator_callback(msg):
    if len(msg.data)==0:
        Sensors['arucolocator'][1] = []
        Sensors['aruco_camera_angle'][1] = 0
        return
    data_superlocator = list(msg.data[0].data)
    datalen = len(data_superlocator)
    Sensors['arucolocator'][1] = data_superlocator
    Sensors['aruco_camera_angle'][1] = datalen//2

def rsaction_callback(cmdmsg):
    global cspeed, _RID_
    global CtlPMove, CtlPOth
    
    cmd = cmdmsg.action
    aid = cmdmsg.agent_id
    if aid!=_RID_: return # Это не мне
    slist = cmdmsg.slist

    arg1 = cmdmsg.arg1
    arg2 = cmdmsg.arg2
    arg3 = cmdmsg.arg3

    # Запрос данных
    #
    # Основные сенсоры
    #
    if(cmd == gdic.CMD_GET_SENS):
        msg_sens = sensors()
        msg_sens.data = []
        msg_sens.src = SERVER_ADDR
        msg_sens.dest = aid
        msg_sens.tm = int(time.time())
        # slist содержит имена запрашиваемых сенсоров
        for sname in slist:
            sname = sname.upper()

            # Датчики TSOP
            '''
            "DET_IR_BEACON_FWD"
            "DET_IR_BEACON_BACK"
            "DET_IR_BEACON_LEFT"
            "DET_IR_BEACON_RIGHT"
            '''

            # С некоторыми именами разбираемся особо
            if sname=='POSITION':
                sensdata = [0,0,Sensors['compass'][1]]
            elif sname=='LOCATOR':
                # Возвращает значения в см. Переводим в единицы Kvorum
                sensdata = [int(r*0.01/cellsize) if r!=math.inf else 0 for r in Sensors['lidar'][1]]
            elif sname=='SUPERLOCATOR':
                # Возвращает значения в см. Переводим в единицы Kvorum
                sensdata = []
                L = len(Sensors['arucolocator'][1])//2
                for i in range(L):
                    dist, code = Sensors['arucolocator'][1][i*2], Sensors['arucolocator'][1][i*2+1]
                    dist = int(dist*0.01/cellsize) if dist!=math.inf else 0
                    sensdata.append(dist)
                    sensdata.append(code)
            elif sname=='ENCODER_LEFT' or sname=='ENCODER_RIGHT':
                sensdata = Sensors[sname.lower()][1]
            else:
                sensdata = []
                # Возвращает значения в см. Переводим в единицы Kvorum
                # Ищем соответствие
                for y13name in Sensors.keys():
                    if Sensors[y13name][0] is None: continue
                    if Sensors[y13name][0].upper()==sname:
                        sensdata = int(Sensors[y13name][1]*0.01/cellsize) if cellsize else 0
            if type(sensdata)!=list: sensdata = [sensdata]
            sval = sdata()
            sval.args = sname
            sval.data = sensdata
            msg_sens.data.append(sval)
        Pub_sensors.publish(msg_sens)
        return
    #
    # Движение
    #
    elif(cmd == gdic.CMD_FWD):
        print(f"-- {aid}: FWD")
        CtlPMove.vlinear, CtlPMove.vangular = cspeed, 0
    elif(cmd == gdic.CMD_BACK):
        print(f"-- {aid}: BACK")
        CtlPMove.vlinear, CtlPMove.vangular = -cspeed, 0
    elif(cmd == gdic.CMD_SET_CURR_V_SPEED):
        CtlPMove.vlinear, CtlPMove.vangular = arg1, math.radians(arg2)
    elif(cmd == gdic.CMD_LEFT or cmd == gdic.CMD_FAST_LEFT):
        print(f"-- {aid}: LEFT")
        CtlPMove.vlinear, CtlPMove.vangular = rotvlin, rotvang # 0.1,  0.05
    elif(cmd == gdic.CMD_RIGHT or cmd == gdic.CMD_FAST_RIGHT):
        print(f"-- {aid}: RIGHT")
        CtlPMove.vlinear, CtlPMove.vangular = rotvlin, -rotvang # 0.1, -0.05
    elif(cmd == gdic.CMD_STOP):
        print(f"-- {aid}: STOP")
        CtlPMove.vlinear, CtlPMove.vangular = 0, 0

    #
    # Прочее
    #

    elif(cmd == gdic.CMD_SET_NORM_V_SPEED):
        cspeed = arg1
    elif(cmd == gdic.CMD_BEEP_ON): # Включить пищалку
        print(f"-- {aid}: BEEP_ON")
        CtlPOth.command = y13cmd.Y13_CMD_BEEP_ON
    elif(cmd == gdic.CMD_BEEP_OFF): # Выключить пищалку
        print(f"-- {aid}: BEEP_OFF")
        CtlPOth.command = y13cmd.Y13_CMD_BEEP_OFF
    elif(cmd == gdic.CMD_F_SET_A_SRC): # Установить излучаемый код
        # Управление маяком:
        # da[0] - y13cmd.subcmdSET_RC5
        # da[1] - Передаваемый код RC5
        # da[2] - Количество разрядов в пакете RC5 (не более 32)
        # da[3] - Частота генерации пакетов, Гц 
        print(f"-- {aid}: F_SET_A_SRC", arg1, arg2, arg3)
        CtlPOth.command = y13cmd.Y13_CMD_USR        
        CtlPOth.da = [y13cmd.subcmdSET_RC5, int(arg2), 16, 5]
    else:
        gdic.warning(f"{aid}: yarp13_kvorum_bridge: unknown command {cmd}")
    '''
    elif(cmd == gdic.CMD_USR_INT): # Пользовательская команда
    '''

def sdvbt_callback(msg):
    global BtCompassData
    BtCompassData = msg.Azimuth

################################################################################
#
# Отправка команд роботу
#
################################################################################

# Управление скоростью
def SendVelCmd(pub):
    global CtlPMove
    velmsg = Twist()
    velmsg.linear.x = CtlPMove.vlinear
    velmsg.angular.z = CtlPMove.vangular
    pub.publish(velmsg)

# Управление сервомашинками и прочими устройствами
def SendCtlCmd(pub):
    if CtlPOth.command==-1: return
    pub.publish(CtlPOth)
    CtlPOth.command = -1

################################################################################
#
# MAIN
#
################################################################################

def main(suffix):
    global _RID_
    global Pub_sensors

    global compass_correction
    global cspeed, rotvlin, rotvang
    global cellsize

    ############################################################################
    # Инициалиизация ROS
    ############################################################################
    rospy.init_node("yarp13_kvorum_bridge"+suffix)

    # Входные топики
    rospy.Subscriber(gdic.TOPIC_NAME_COMMAND, action, rsaction_callback, queue_size = 3)

    rospy.Subscriber('yy_sensors'+suffix, yy_sens, robot_data_callback, queue_size = 3)
    rospy.Subscriber('superlocator'+suffix, sensors, data_superlocator_callback, queue_size = 3)
    rospy.Subscriber('bt_data'+suffix, bt_data, sdvbt_callback, queue_size = 3)

    # Выходные топики
    pub_cmd = rospy.Publisher('yy_command'+suffix, yy_cmd, queue_size = 3)
    pub_vel = rospy.Publisher('cmd_vel'+suffix, Twist, queue_size = 3)

    Pub_sensors = rospy.Publisher(gdic.TOPIC_NAME_SENSORS, sensors, queue_size = 3)

    try:
        rospy.get_param_names()
    except ROSException:
        print("could not get param name")

    compass_correction = int(rospy.get_param("~compass", 0))

    cspeed = rospy.get_param("~cspeed", 0.25)
    rotvlin = rospy.get_param("~rotvlin", 0.05)
    rotvang = rospy.get_param("~rotvang", 0.1)

    cellsize = rospy.get_param("~cellsize", 0.1)

    print(f":: compass_correction={compass_correction}")
    print(f":: cspeed={cspeed}, rotvlin={rotvlin}, rotvang={rotvang}, cellsize={cellsize}")

    #####################################################
    # Основной цикл
    #####################################################
    print("Start main loop")

    rate = rospy.Rate(10) # 10 Hz

    while not rospy.is_shutdown():
        # Будем с частотой 10 Гц отправлять команды в любом случае, даже повторяя старые

        # Управление скоростью
        SendVelCmd(pub_vel)

        # Управление сервомашинками и прочими устройствами
        SendCtlCmd(pub_cmd)

        rate.sleep()

    gdic.terminate_program()

################################################################################
#
################################################################################
if __name__ == '__main__':

    print(Title)
    parser = argparse.ArgumentParser(prog = '', description = Title, epilog = '')
    parser.add_argument('--rid', nargs='?', type=str, default="", required=False, metavar = 'robot_id', help = 'robot id (id=1,2...)')

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

    _RID_ = 1 if namespace.rid=="" else int(namespace.rid)

    main(namespace.rid)
