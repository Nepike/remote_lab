#!/usr/bin/env python3
# coding: utf-8
'''
Version 1.1

Прием-передача файла маршрута по XBee

16.03.2025

LP 16.03.2025
'''

import sys, os, argparse
import time

from zbconnection import ZigBeeConnection
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__))+"/../lib")
import rdcomm

Title = "\nXBee Communication 1.1\n"

CMD_START = "<start>"
CMD_STOP = "<stop>"

def warning(msg):
    print(f"*** Warning: {msg}")

num_errors = 0
def SendMsg(xdev,msg, recepient):
    MAX_REP = 5
    cnt = 1
    while True:
        r = xdev.transmit_msg(msg, recepient)
        time.sleep(0.1)
        print(f"{recepient}, #{cnt}: {r}")
        if 1 in r: break
        num_errors += 1
        # Ошибка. Будем повторять
        if cnt>MAX_REP: break
        cnt+=1

#
# Читаем файл и отправляем
#
def TransmitFile(xdev, fname, recepient):
    print(f"Transimt file {fname}")
    if fname is None:
        warning("ReadAndTransmit: fname is None")
        return
    if recepient is None:
        warning("ReadAndTransmit: recepient is None")
        return

    start_time = time.time()
    f = open(fname, 'r', encoding='utf-8')
    text = f.read()
    f.close()
    text = text.split('\n')
    # Передаем
    SendMsg(xdev, CMD_START, recepient)
    for s in text:
        msg = s.strip()
        SendMsg(xdev, msg, recepient)
    SendMsg(xdev, CMD_STOP, recepient)

    end_time = time.time()
    elapsed_time = end_time - start_time
    print("Done.")
    print(f"Elapsed time: {elapsed_time}")
    print(f"Errors: {num_errors}")

#
# Ждем start, принимаем и записываем до тех пор, пока на придет сообщение 'stop'
#
def ReadFile(xdev, fname):
    print("Read File. Wait data...")
    f = open(fname, 'w', encoding='utf-8')
    # Ждем маркер
    while(True):
        msg = xdev.get_message()
        if msg is None: continue
        msg = msg.strip()
        if msg==CMD_START: break

    start_time = time.time()
    while(True):
        msg = xdev.get_message()
        if msg is None: continue
        msg = msg.strip()
        if msg==CMD_STOP: break
        print(msg)
        print(msg, file=f)
    f.close()

    end_time = time.time()
    elapsed_time = end_time - start_time
    print("Done.")
    print(f"Elapsed time: {elapsed_time}")

if __name__ == '__main__':

    parser = argparse.ArgumentParser(prog = sys.argv[0], description = Title, epilog = '')

    parser.add_argument('--device', nargs='?', default=None, required=True, metavar = 'device', help = 'xbee device (/dev/ttyUSBx)')
    parser.add_argument('--addr', type=int, nargs='?', default=None, required=True, metavar = 'my_addr', help = 'my address')
    parser.add_argument('--recepient', type=int, nargs='?', default=None, required=False, metavar = 'rec_addr', help = 'recepient address')
    parser.add_argument ('--input', default=None, required=False, type=str, metavar = 'inpfile', help = 'file to trasmit')
    parser.add_argument ('--output', default=None, required=False, type=str, metavar = 'outfile', help = 'file to save')

    namespace = parser.parse_args(sys.argv[1:])

    # Создаем устройство
    xdev = ZigBeeConnection(namespace.addr, namespace.device)

    print(Title)

    # Читаем и передаем файл
    if not (namespace.input is None):
        TransmitFile(xdev, namespace.input, namespace.recepient)

    # Читаем и записываем
    if not(namespace.output is None):
        ReadFile(xdev, namespace.output)

    # Слушаем
    print("Listening...")
    while(True):
        msg = xdev.get_message()
        if msg is None: continue
        msg = msg.strip()
        if msg==CMD_STOP: break
        print(msg)

