#!/usr/bin/env python3
# coding: utf-8

"""
Version 1.1

Прием-передача файла маршрута по RC5

16.03.2025

LP 16.03.2025

input:  msg_yy.msg.sens
output: msg_yy.msg.cmd

"""

import time
from datetime import datetime
import sys, os, rospy
import argparse
from threading import Thread

#sys.path.insert(0, os.path.dirname(os.path.realpath(__file__))+"/../lib")

from yyctl import y13cmd
from msg_yy.msg import sens as yy_sens
from msg_yy.msg import cmd as yy_cmd

from yyctl import rdcomm

Title = "\nRC5 Communication 1.1\n"

pub_cmd = None

_RID_ = ""
CMD_TOPIC_NAME = 'yy_command'
SENS_TOPIC_NAME = 'yy_sensors'
NODE_NAME = 'trc_node'

OutputFile = None
ProgramTerminated = False

CmdMsg = yy_cmd()
CmdMsg.angle = [0,0,0]
CmdMsg.arg = [0,0,0]
CmdMsg.command = 0


def data_cb(msg):
    #print(msg)
    EvalIRData(msg.irdata)
    return

def readstr(prompt):
    try:
        if sys.version_info.major<3:
            s = raw_input(prompt)
        else:
            s = input(prompt)
    except:
        s = ""
    return s

PackageData = []
pred_package = []

CNT = 0

def EvalIRData(data):
    global CNT
    global pred_package, PackageData
    global ProgramTerminated
    nd = [e for e in data if e!=0]
    if len(nd)==0: return
    nd = list(set(nd))
    if len(nd)!=1: return
    nd = nd[0]
    print(f"{CNT}: nd={nd} {nd:0x}", end='')

    data, _, _ = rdcomm.unpack_data(nd)
    if not (data is None):
        print(f"  => {data}", end='')
        if pred_package != data:
            PackageData.append(data)
            pred_package = data
            # Не пора ли заканчивать?
            if data[0] == rdcomm.TIP_CMD and data[1]==rdcomm.CMD_STOP:
                rdcomm.SaveSentences(OutputFile, PackageData)
                PackageData = []
                ProgramTerminated = True
    print()

    CNT+=1
    if ProgramTerminated:
        print("-- process finished")

def SendData(data, size, freq):
    global pub_cmd
    CmdMsg.command = y13cmd.Y13_CMD_USR
    CmdMsg.da[0] = y13cmd.subcmdSET_RC5 # Параметры маяка RC5
    CmdMsg.da[1] = data                 # Передаваемый код RC5
    CmdMsg.da[2] = size                 # Количество разрядов в пакете RC5 (не более 32)
    CmdMsg.da[3] = freq                 # Частота генерации пакетов, Гц
    pub_cmd.publish(CmdMsg)

def TransmitFile(inpfile):
    global pub_cmd
    Tsend = 3
    Freq = 5

    # Читаем из файла
    try:
        sentences = rdcomm.ReadSentenсes(inpfile)
    except:
        rdcomm.error(f"rdcomm.ReadSentenсes({inpfile})")

    print("Source sentences:")
    print(sentences)

    # Формируем двоичное представление для отправки по RC5
    data = rdcomm.PackSentences(sentences)

    print("RC5 Data:")
    print(data)

    # Отправка
    time.sleep(Tsend)

    rate = rospy.Rate(50)

    for d in data:
        print(f"send {d}")
        n, size = d[0], d[1]
        SendData(n, size, freq=Freq)

        time.sleep(Tsend)

        rate.sleep()

    SendData(0,0,0)

    pub_cmd.publish(CmdMsg)
#
#
#
def main(inpfile, noctl):
    global pub_cmd
    global ProgramTerminated

    print(Title)

    print(f"NODE = {NODE_NAME}")
    print(f"CMD_TOPIC = {CMD_TOPIC_NAME}")
    print(f"SENS_TOPIC = {SENS_TOPIC_NAME}")

    rospy.init_node(NODE_NAME)

    rospy.Subscriber(SENS_TOPIC_NAME, yy_sens, data_cb)
    pub_cmd = rospy.Publisher(CMD_TOPIC_NAME, yy_cmd, queue_size = 2)

    if inpfile:
        TransmitFile(inpfile)
        sys.exit(0)

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        if ProgramTerminated: break
        if not noctl:
            s = readstr(">")
            if s in ['Q', 'q']: break
            elif s in ['1']: SendData(0x1, size = 30,freq = 5)
            elif s in ['2']: SendData(0x2, size = 30,freq = 5)
            elif s in ['3']: SendData(0x3, size = 30,freq = 5)
            elif s in ['4']: SendData(0x4, size = 30,freq = 5)
            elif s in ['9']: SendData(0x12345678, size = 30,freq = 5)
            elif s in ['0']:
                SendData(0, 0, 0)

        pub_cmd.publish(CmdMsg)

        rate.sleep()

if __name__ == "__main__" :

    parser = argparse.ArgumentParser(prog = '',  description = Title,  epilog = '')

    parser.add_argument ('-r', '--rid', default="", required=False, type=str, metavar = 'robit_id', help = 'robot id')

    parser.add_argument ('--input', default=None, required=False, type=str, metavar = 'inpfile', help = 'file to trasmit')
    parser.add_argument ('--output', default=None, required=False, type=str, metavar = 'outfile', help = 'file to save')

    parser.add_argument('--noctl', action='store_const', const=True, default=False, help = 'disable ctl regime')

    #parser.add_argument('--reg', nargs='?', choices=['create', 'exec'], default=None, required=True, metavar = 'create | exec', help = 'create or exec database')

    namespace = parser.parse_args(sys.argv[1:])

    _RID_ = namespace.rid
    CMD_TOPIC_NAME = CMD_TOPIC_NAME+_RID_
    SENS_TOPIC_NAME = SENS_TOPIC_NAME+_RID_
    NODE_NAME = NODE_NAME+_RID_
    OutputFile = namespace.output

    main(namespace.input, namespace.noctl)
