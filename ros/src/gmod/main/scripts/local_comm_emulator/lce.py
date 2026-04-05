#!/usr/bin/env python3
#coding: utf-8
'''
Эмулятор локальной связи
По мотивам программы А.А.Малышева

Все углы здесь в градусах

V 1.1
28.12.2024

LP 10.01.2025
'''

import sys,time
import rospy
import tf
import numpy as np
from std_msgs.msg import Int32, Float32, Int32MultiArray,Float32MultiArray, MultiArrayDimension
from gazebo_msgs.msg import ModelStates
import math
import argparse
import configparser

from msg_kvorum2.msg import action
from msg_kvorum2.msg import sdata, sensors
from kvorum2 import gdic

Title = 'Local Communication Emulator 1.1'

rc5_dist = None      # Дистанция обнаружения
output_topic = None  # Имя выходного топика

eps = 10          # Погрешность определения угла (+-eps)

Q_FWD = 0         # Нумерация TSOP
Q_BCK = 1
Q_LF  = 2
Q_RT  = 3

# Префикс имени агента
agent_prefix = 'mobot_mobot'

class TRC5(object):
    def __init__(self, name):
        self.x = 0 # Координаты
        self.y = 0
        self.a = 0    # Угол (в градусах)
        self.id = 0   # id агента
        self.name = name
        self.code = 0 # Излучаемый код
        self.val = [0,0,0,0] # Значения датчиков (приемников TSOP)

Agents = {}

def yaw_from_quaternion_msg(q_msg):
    qu = [q_msg.x, q_msg.y, q_msg.z, q_msg.w]
    return tf.transformations.euler_from_quaternion(qu)[2]

def anorm(a):
    a = a%360
    if a<0: a += 360
    return int(a)

def is_robot_name(name):
    return name.find(agent_prefix, 0)==0

def cb_model_states(msg):
    for name in msg.name:
        if is_robot_name(name):
            n = msg.name.index(name)
            angle = yaw_from_quaternion_msg(msg.pose[n].orientation)
            x = msg.pose[n].position.x
            y = msg.pose[n].position.y
            if not (name in Agents):
                Agents[name] = TRC5(name)
                Agents[name].id = int(name[len(agent_prefix):])
            Agents[name].x = x
            Agents[name].y = y
            Agents[name].a = int(math.degrees(angle))
            # Это для автономной отладки
            #Agents[name].code = Agents[name].id

def cb_rsaction(msg):
    cmd = msg.action
    aid = msg.agent_id

    level = int(msg.arg1) # level
    code = int(msg.arg2)  # value
    if(cmd != gdic.CMD_F_SET_A_SRC): return
    # Задаем код
    for n in Agents.keys():
        if Agents[n].id==aid:
            Agents[n].code = code
            print(f"-- {Agents[n].name} {Agents[n].id} {Agents[n].code}")


def Calculate():
    for n1 in Agents.keys():
        Agents[n1].val = [0, 0, 0, 0]
        cv = [[],[],[],[]]
        p1 = np.array([Agents[n1].x, Agents[n1].y])
        for n2 in Agents.keys():
            if n1==n2: continue
            p2 = np.array([Agents[n2].x, Agents[n2].y])
            dist = np.linalg.norm(p1-p2)
            if dist>rc5_dist: continue
            t = int(math.degrees(np.arctan2((p2[1]-p1[1]),(p2[0]-p1[0]))))
            a = anorm(t-Agents[n1].a)
            code = Agents[n2].code

            '''
            if a<=45+eps or a>=315-eps: Agents[n1].val[0] = code
            if a>=45-eps and a<=135+eps: Agents[n1].val[2] = code
            if a>=135-eps and a<=225+eps: Agents[n1].val[1] = code
            if a>=225-eps and a<=315+eps: Agents[n1].val[3] = code
            '''
            if a<=45+eps or a>=315-eps: cv[0].append((code, dist))
            if a>=45-eps and a<=135+eps: cv[2].append((code, dist))
            if a>=135-eps and a<=225+eps: cv[1].append((code, dist))
            if a>=225-eps and a<=315+eps: cv[3].append((code, dist))
        # Оставляем ближайшие
        for n in range(len(cv)):
            if len(cv[n])==0: mc, md = 0, 0
            else:
                _c = [e[0] for e in cv[n]]
                _d = [e[1] for e in cv[n]]
                md = min(_d)
                i = _d.index(md)
                mc = _c[i]
            Agents[n1].val[n] = mc

        print(f"{n1} a={Agents[n1].a} code={Agents[n1].code} val={Agents[n1].val}")
    return

def main(configfile):

    global rc5_dist, output_topic

    config = configparser.ConfigParser()
    config.read(configfile)

    rc5_dist = float(config['LCERC5']['rc5_dist'])
    output_topic = config['LCERC5']['output_topic']
    
    # Источник -- Gazebo:
    rospy.Subscriber('/gazebo/model_states', ModelStates, cb_model_states)

    # Источник -- KVORUM:
    rospy.Subscriber(gdic.TOPIC_NAME_COMMAND, action, cb_rsaction, queue_size = 1)

    pub = rospy.Publisher(output_topic, sensors, queue_size = 1)

    rate = rospy.Rate(50)

    msg_sens = sensors()
    msg_sens.src = 0
    msg_sens.dest = 0
    while not rospy.is_shutdown():
        Calculate()

        # Публикуем
        msg_sens.data = []
        msg_sens.tm = int(time.time())
        for n in Agents.keys():
            sval = sdata()
            sval.args = str(Agents[n].id)
            sval.data = Agents[n].val
            msg_sens.data.append(sval)
        pub.publish(msg_sens)

        rate.sleep()

if __name__ == '__main__':

    print(Title)
    parser = argparse.ArgumentParser(prog = '', description = Title, epilog = '')
    parser.add_argument('--cfg', nargs='?', default=None, required=True, metavar = 'filename', help = 'topics description filename')

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

    rospy.init_node('local_comm_emulator')
    main(namespace.cfg)
