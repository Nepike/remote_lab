#!/usr/bin/env python3
"""
    A simple test server for bluetooth positioning data acquisition from a mobile device
    https://github.com/pybluez/pybluez/blob/master/examples/simple/rfcomm-server.py
    Mod by Petr Sorokoumov
    V 1.01
    11.01.2023
   
"""

import sys, os, glob
import time, math
import random

from bluetooth import *

field_names = ['AccX', 'AccY', 'AccZ',           # acceleration
               'AngVelX', 'AngVelY', 'AngVelZ',  # angular velocity
               'Azimuth', 'Pitch', 'Roll',       # orientation (azimuth is compass value)
               'Distance',                       # proximity (whatever it is)
               'Latitude', 'Longitude', 'Altitude', 'Speed']  # global position and speed

# Convert input byte sequence to dictionary
def parse_message(msg_bytes):
    # convert input to string
    msg_str = msg_bytes.decode('utf-8')
    # convert string to list of floats
    try:
        raw_seq = [float(x) for x in msg_str.strip('()\n').split(',')]
    except ValueError:
        print('Incorrect value in line ' + msg_str)
        return {}
    # check length
    if len(field_names) < len(raw_seq):
        print('There are {} extra field(s) in input line '.format(
            len(raw_seq) - len(field_names)) + msg_str)
    elif len(field_names) > len(raw_seq):
        print('There are no {} necessary field(s) in input line '.format(
            len(field_names) - len(raw_seq)) + msg_str)
    return {n : v for n, v in zip(field_names, raw_seq)}

# create socket
server_sock = BluetoothSocket( RFCOMM )
server_sock.bind(("",PORT_ANY))
server_sock.listen(1)

# get port 
port = server_sock.getsockname()[1]

# get arbitrary uuid to identify service
uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ff"

# trying to register service in system
print('start advertise {}'.format(port))
advertise_service( server_sock, "TestServer",
                   service_id = uuid,
                   service_classes = [ uuid, SERIAL_PORT_CLASS ],
                   profiles = [ SERIAL_PORT_PROFILE ])

# waiting for connection
print( "Waiting for connection on RFCOMM channel %d" % port)
client_sock, client_info = server_sock.accept()
print( "Accepted connection from ", client_info)

while True:

    try:
        # receive data
        raw_input = client_sock.recv(1024)
        if len(raw_input) == 0:
            break
        # parse data
        sensor_data = parse_message(raw_input)
        # print it
        print( "received [%s]" % raw_input)
        print(sensor_data)

    except IOError:
        pass

    except KeyboardInterrupt:

        print ("disconnected")

        client_sock.close()
        server_sock.close()
        print ("all done")

        break
