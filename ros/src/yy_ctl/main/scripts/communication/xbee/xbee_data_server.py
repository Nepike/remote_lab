#!/usr/bin/env python3
# coding: utf-8
'''
Version 1.1

Сервер передачи данных по ZigBee
Запускается там же, где живет устройство

16.03.2025

LP 18.03.2025
'''

import sys, os, argparse
import time

from yyctl.zbconnection import ZigBeeConnection
import roslib, rospy

from msg_yy.msg import dtm

import xbeedslib

_RID_ = ""

Title = "\nXBee Server 1.1\n"

CommandMessage = None

num_errors = 0
def ZigBeeSendMsg(xdev,msg, recepient):
    global num_errors
    MAX_REP = 5
    cnt = 1
    while True:
        r = xdev.transmit_msg(msg, recepient)
        time.sleep(0.5)
        print(f"{recepient}, #{cnt}: {r}")
        if 1 in r: break
        num_errors += 1
        # Ошибка. Будем повторять
        if cnt>MAX_REP: break
        cnt+=1

#
# Отправляем данные
#
def TransmitData(xdev, data, recepient):
    print(f"Transimt data to {recepient}")
    if data is None:
        xbeedslib.warning("TransmitData: data is None")
        return
    if recepient is None:
        xbeedslib.warning("TransmitData: recepient is None")
        return

    start_time = time.time()
    text = data.split('\n')
    # Передаем
    ZigBeeSendMsg(xdev, xbeedslib.MARKER_START, recepient)
    for s in text:
        msg = s.strip()
        ZigBeeSendMsg(xdev, msg, recepient)
    ZigBeeSendMsg(xdev, xbeedslib.MARKER_STOP, recepient)

    end_time = time.time()
    elapsed_time = end_time - start_time
    print(f"Elapsed time: {elapsed_time}")
    print(f"Errors: {num_errors}")

#
# Читаем файл и отправляем
#
def TransmitFile(xdev, fname, recepient):
    print(f"Transimt file {fname}")
    if fname is None:
        xbeedslib.warning("ReadAndTransmit: fname is None")
        return
    if recepient is None:
        xbeedslib.warning("ReadAndTransmit: recepient is None")
        return
    f = open(fname, 'r', encoding='utf-8')
    text = f.read()
    f.close()
    TransmitData(xdev, text, recepient)
    print("Done.")

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
    data = None
    while(True):
        msg = xdev.get_message()
        if msg is None: continue
        msg = msg.strip()
        print(f":{msg}")
        if msg==xbeedslib.MARKER_STOP: break    
        if data is None: data = msg
        else: data = data + "\n" + msg
    end_time = time.time()
    elapsed_time = end_time - start_time
    print(f"Elapsed time: {elapsed_time}")
    return data

def ReadData2File(xdev, fname):
    print("Read and write to file {fname}")
    data = ReadData(xdev)
    f = open(fname, 'w', encoding='utf-8')
    for e in data:
        print(e, file=f)
    f.close()
    print("Done.")
    return data

################################################################################

def cmd_callback(msg):
    global CommandMessage
    CommandMessage = msg

def Publish(pub, cmd, data):
    msg = dtm()    
    msg.cmd = cmd
    msg.arg1 = 0
    msg.arg2 = 0
    msg.data = data
    pub.publish(msg)

################################################################################
#
#
#
################################################################################
if __name__ == '__main__':

    parser = argparse.ArgumentParser(prog = sys.argv[0], description = Title, epilog = '')

    parser.add_argument('--device', nargs='?', default=None, required=True, metavar = 'device', help = 'xbee device (/dev/ttyUSBx)')
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

    # Создаем устройство
    xdev = ZigBeeConnection(namespace.addr, namespace.device)
    _RID_ = str(namespace.addr) if namespace.addr>0 else ""

    print(Title)

    # Читаем и передаем файл
    if not (namespace.input is None):
        TransmitFile(xdev, namespace.input, namespace.recepient)
        sys.exit(0)

    # Читаем и записываем
    if not(namespace.output is None):
        ReadData2File(xdev, namespace.output)
        sys.exit(0)

    #rospy.init_node("xbee_data_server"+_RID_)
    rospy.init_node("xbee_data_server"+_RID_)

    # Входной топик
    rospy.Subscriber(xbeedslib.command_topic_name+_RID_, dtm, cmd_callback)

    # Выходной топик
    pub_res = rospy.Publisher(xbeedslib.result_topic_name+_RID_, dtm, queue_size=10)

    print(f"{_RID_}: Listening...")

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():

        if CommandMessage:
            # Команды для передатчка
            if CommandMessage.cmd==xbeedslib.CMD_SEND_DATA:
                Publish(pub_res, cmd = xbeedslib.STAT_START_TRANSFERING, data = "START_TRANSFERING")
                rate.sleep()
                TransmitData(xdev, CommandMessage.data, recepient=CommandMessage.arg1)
                Publish(pub_res, cmd = xbeedslib.STAT_END_TRANSFERING, data = "END_TRANSFERING")
                rate.sleep()
            elif CommandMessage.cmd==xbeedslib.CMD_SEND_FILE:
                Publish(pub_res, cmd = xbeedslib.STAT_START_TRANSFERING, data = "START_TRANSFERING")
                rate.sleep()
                TransmitFile(xdev, CommandMessage.data, recepient=CommandMessage.arg1)
                Publish(pub_res, cmd = xbeedslib.STAT_END_TRANSFERING, data = "END_TRANSFERING")
                rate.sleep()
            # Команды для приемника
            elif CommandMessage.cmd==xbeedslib.CMD_READ_DATA:
                Publish(pub_res, cmd = xbeedslib.STAT_WAIT_FOR_DATA, data = "STAT_WAIT_FOR_DATA")
                rate.sleep()
                data = ReadData(xdev)
                Publish(pub_res, cmd = xbeedslib.STAT_DATA_READY, data = str(data))
                rate.sleep()
            elif CommandMessage.cmd==xbeedslib.CMD_READ_DATA_FILE:
                Publish(pub_res, cmd = xbeedslib.STAT_WAIT_FOR_DATA, data = "STAT_WAIT_FOR_DATA")
                rate.sleep()
                data = ReadData2File(xdev, CommandMessage.data)
                Publish(pub_res, cmd = xbeedslib.STAT_DATA_FILE_READY, data = str(data))
                rate.sleep()
            else:
                xbeedslib.error(f"unknown command {CommandMessage}")
            CommandMessage = None
            print(f"{_RID_}: Listening...")

        # Слушаем
        msg = xdev.get_message()
        if not (msg is None):
            msg = msg.strip()
            print(msg)
        rate.sleep()
