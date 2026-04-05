#!/usr/bin/env python3
# coding: utf-8
'''
Version 1.1

Сервер передачи данных по ZigBee
Управляет сервером ZigBee через топики

16.03.2025

LP 18.03.2025
'''

import sys, os, argparse
import time

import roslib, rospy

from msg_yy.msg import dtm

import xbeedslib

_RID_ = ""

ProgramTerminated = False

Title = "\nXBee Data Server 1.1\n"


Regime = None


StatusMessage = None

OutputFileName = None

#
# Ждем start, принимаем и записываем до тех пор, пока на придет сообщение 'stop'
#
def ReadData(xdev):
    print("Read Data...")
    # Ждем маркер
    while(True):
        msg = xdev.get_message()
        if msg is None: continue
        msg = msg.strip()
        if msg==xbeedslib.MARKER_START: break

    start_time = time.time()
    data = []
    while(True):
        msg = xdev.get_message()
        if msg is None: continue
        msg = msg.strip()
        print(msg)
        if msg==xbeedslib.MARKER_STOP: break
        data.append(msg)
    end_time = time.time()
    elapsed_time = end_time - start_time
    print(f"Elapsed time: {elapsed_time}")
    return data

################################################################################

def cmd_callback(msg):
    global StatusMessage, ProgramTerminated, OutputFileName
    StatusMessage = msg
    print(f"read: status={StatusMessage.cmd} {StatusMessage.data}")
    if Regime == xbeedslib.REG_READER:
        if (StatusMessage.cmd == xbeedslib.STAT_DATA_READY) or (StatusMessage.cmd == xbeedslib.STAT_DATA_FILE_READY):
            ProgramTerminated = True
    if Regime == xbeedslib.REG_FILE_READER:
        if (StatusMessage.cmd == xbeedslib.STAT_DATA_READY) or (StatusMessage.cmd == xbeedslib.STAT_DATA_FILE_READY):
            WriteData2File(OutputFileName, StatusMessage.data)
            ProgramTerminated = True
    if Regime == xbeedslib.REG_SENDER:
        if (StatusMessage.cmd == xbeedslib.STAT_END_TRANSFERING):
            ProgramTerminated = True

def Publish(pub, cmd, arg1, arg2, data):
    msg = dtm()
    msg.cmd = cmd
    msg.arg1 = arg1
    msg.arg2 = arg2
    msg.data = data
    print(f"publish: {msg.cmd} {msg.data}")
    pub.publish(msg)

#
# Читаем файл и отправляем
#
def TransmitFile(pub, fname, recepient):
    print(f"Transimt file {fname}")
    f = open(fname, 'r', encoding='utf-8')
    data = f.read()
    f.close()
    Publish(pub, cmd=xbeedslib.CMD_SEND_DATA, arg1=recepient, arg2=0, data=data)
    print("Done.")

def WriteData2File(fname, data):
    print(f"Write to file {fname}")
    f = open(fname, 'w', encoding='utf-8')
    print(data, end='', file=f)
    f.close()
    print("Done.")

################################################################################
#
#
#
################################################################################
if __name__ == '__main__':

    parser = argparse.ArgumentParser(prog = sys.argv[0], description = Title, epilog = '')

    parser.add_argument('--addr', type=int, nargs='?', default=None, required=True, metavar = 'my_addr', help = 'my address')

    parser.add_argument('--recepient', type=int, nargs='?', default=None, required=False, metavar = 'rec_addr', help = 'recepient address')
    parser.add_argument ('--input', default=None, required=False, type=str, metavar = 'inpfile', help = 'file to trasmit')
    parser.add_argument ('--output', default=None, required=False, type=str, metavar = 'outfile', help = 'file to save')

    # Странный финт
    # Первый и два последних аргумента (__name, __log) не нужны
    # Сделан потому, что при запуске roslaunch добавляются агрументы "__name:=...", и "__log:=..."
    arglist = []
    for e in sys.argv:
        if e.find("__name:=")==0: continue
        if e.find("__log:=")==0: continue
        arglist.append(e)
    namespace = parser.parse_args(arglist[1:])

    _RID_ = str(namespace.addr) if namespace.addr>0 else ""

    if not (namespace.input is None) and not (namespace.recepient is None): Regime = xbeedslib.REG_SENDER
    elif not(namespace.output is None):
        Regime = xbeedslib.REG_FILE_READER
        OutputFileName = namespace.output
    else: Regime = xbeedslib.REG_READER

    print(Title)

    rospy.init_node("xbee_ds"+_RID_)

    # Входной топик
    rospy.Subscriber(xbeedslib.result_topic_name+_RID_, dtm, cmd_callback)

    # Выходной топик
    pub = rospy.Publisher(xbeedslib.command_topic_name+_RID_, dtm, queue_size=10)

    rate = rospy.Rate(50)
    time.sleep(1)

    enable = True
    while not rospy.is_shutdown():

        # Передаем файл
        if Regime == xbeedslib.REG_SENDER and enable:
            TransmitFile(pub, namespace.input, namespace.recepient)
            enable = False

        # Читаем [и записываем в файл]
        if (Regime==xbeedslib.REG_READER or Regime==xbeedslib.REG_FILE_READER) and enable:
            Publish(pub, cmd=xbeedslib.CMD_READ_DATA, arg1=0, arg2=0, data="")
            enable = False

        if ProgramTerminated: break

        rate.sleep()
