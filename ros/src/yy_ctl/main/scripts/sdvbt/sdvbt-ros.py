#!/usr/bin/env python3
# coding: utf-8
"""
    A simple test server for bluetooth positioning data acquisition from a mobile device
    https://github.com/pybluez/pybluez/blob/master/examples/simple/rfcomm-server.py
    Mod by Petr Sorokoumov
    V 1.01
    11.01.2023

    node = 'bt_data_node'
    Данные публикуются в топик 'bt_data'

"""

import sys, os, time
import math
import rospy

from msg_yy.msg import bt_data

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

rospy.init_node('bt_data_node')
pub_data = rospy.Publisher('bt_data', bt_data, queue_size = 1)

msg = bt_data()

rate = rospy.Rate(100)

while not rospy.is_shutdown():

    try:
        # receive data
        raw_input = client_sock.recv(1024)
        if len(raw_input) == 0:
            break

    except IOError:
        pass

    except KeyboardInterrupt:
        print ("disconnected")
        client_sock.close()
        server_sock.close()
        print ("all done")
        break

    # parse data
    sensor_data = parse_message(raw_input)
    # print it
    print( "received [%s]" % raw_input)
    print(sensor_data)

    # Публикуем
    msg.AccX = sensor_data['AccX']
    msg.AccY = sensor_data['AccY']
    msg.AccZ = sensor_data['AccZ']

    # Angular velocity
    msg.AngVelX = sensor_data['AngVelX']
    msg.AngVelY = sensor_data['AngVelY']
    msg.AngVelZ = sensor_data['AngVelZ']

    # Orientation (azimuth is compass value)
    msg.Azimuth = sensor_data['Azimuth']
    msg.Pitch = sensor_data['Pitch']
    msg.Roll = sensor_data['Roll']

    msg.Distance = sensor_data['Distance'] # proximity (whatever it is)

    # Global position and speed
    msg.Latitude = sensor_data['Latitude']
    msg.Longitude = sensor_data['Longitude']
    msg.Altitude = sensor_data['Altitude']
    msg.Speed = sensor_data['Speed']

    msg.tm = rospy.Time.now()
    pub_data.publish(msg)

    rate.sleep()
