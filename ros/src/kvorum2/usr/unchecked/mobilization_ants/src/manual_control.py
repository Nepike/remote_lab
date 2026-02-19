#!/usr/bin/env python
# coding: utf-8
"""
  manual_control.py

  Manual control (movement) for an ant in Kvorum.

  @author Rovbo Maxim
  @version 1.0
  @date 2.06.2018
"""
import sys, tty, termios, re

import rospy

import rcproto
import rcproto_helper as rcx

from msg_rsaction.msg import action

def init_system() :
    global cmd_pub
    global robot_id

    rospy.init_node("manual_control")

    robot_id = rospy.get_param('~robot_id', 1)

    # Выходной топик
    cmd_pub = rospy.Publisher("/actions_topic", action,
            queue_size=10)

    rcx.set_cmd_pub(cmd_pub)

def send(cmd) :
    global robot_id

    if cmd == ' ' :
        rcx.send_cmd(robot_id, rcproto.CMD_STOP)
    elif cmd == 'w' :
        rcx.send_cmd(robot_id, rcproto.CMD_FWD)
    elif cmd == 'a' :
        rcx.send_cmd(robot_id, rcproto.CMD_FAST_LEFT)
    elif cmd == 'd' :
        rcx.send_cmd(robot_id, rcproto.CMD_FAST_RIGHT)
    elif cmd == 's' :
        rcx.send_cmd(robot_id, rcproto.CMD_BACK)
    else :
        pass

def control_cb(event) :
    global cmd
    global last_cmd

    if last_cmd == cmd :
        return

    # transmit the new command
    send(cmd)

    last_cmd = cmd

def getch() :
    """ Read a key.

    Source: http://code.activestate.com/recipes/134892/
    """
    fd = sys.stdin.fileno()

    old_settings = termios.tcgetattr(fd)
    try :
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally :
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    return ch

def main():
    global last_cmd
    global cmd

    last_cmd = 0
    cmd = 0

    rospy.Timer(rospy.Duration(0.1), control_cb)

    # get cmd from user
    while not rospy.is_shutdown():
        cmd = getch()
        print(cmd)

        if not re.match('^\w|\s$', cmd) :
            print('Non-alphanumeric character: exiting')
            exit(1)


if __name__ == '__main__':
    print("Manual ant control")

    init_system()
    main()

    print("Finished successfully")
