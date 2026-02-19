#!/usr/bin/env python
# coding: utf-8
"""
  simple_ants.py

  Ants mobilization in a foraging problem for Kvorum.

  @author Rovbo Maxim
  @version 1.0
  @date 29.05.2018
"""
from __future__ import division

import os, sys, time
import roslib, rospy, random
import numpy

import rcproto
import rcproto_helper as rcx

from msg_rsaction.msg import action
from msg_ans.msg import ans

# import agents.agents as agents
import agents.fsm_agents as agents


class Robot(object) :
    """
    Robot interface abstraction.
    """

    # actions
    ACTION_STOP = 0
    ACTION_GO_FWD = 1
    ACTION_TURN_LEFT_10 = 2
    ACTION_TURN_RIGHT_10 = 3


    def __init__(self, id, controller) :
        self.id = id
        self.controller = controller


    def update_sensors(self) :
        global cmd_pub

        # request update
        # TODO: more than GET_SENS might be needed
        msg = action()
        msg.team_id = 1
        msg.agent_id = self.id
        msg.action = rcproto.CMD_GET_SENS
        msg.data = []

        cmd_pub.publish(msg)

        # TODO: maybe wait for a reply and get data from it
        # currently not waiting and using data that we have

    def step(self) :
        """ Perform an action.
        """
        action = self.controller.choose_action(None)

        # do function from a dictionary of actions
        if action == self.ACTION_STOP:
            self.action_stop()
        elif action == self.ACTION_GO_FWD:
            self.action_go_fwd()
        elif action == self.ACTION_TURN_LEFT_10:
            self.action_turn_left_10()
        elif action == self.ACTION_TURN_RIGHT_10:
            self.action_turn_right_10()
        else:
            raise ValueError('Unrecognized action requested')

    def action_stop(self) :
        rcx.send_cmd(self.id, rcproto.CMD_STOP)

    def action_go_fwd(self) :
        rcx.send_cmd(self.id, rcproto.CMD_FWD)

    def action_move_to_landmark(self, landmark) :
        pass

    def action_get_food(self, food_source) :
        pass

    def action_put_food(self, ) :
        pass

    def action_turn_left_10(self):
        rcx.send_cmd(self.id, rcproto.CMD_FAST_LEFT, 10)

    def action_turn_right_10(self):
        rcx.send_cmd(self.id, rcproto.CMD_FAST_RIGHT, 10)


def data_cb(msg) :
    """ Process received information from the robot or its
    simulated counterpart.
    """
    robot_id = msg.data[rcproto.POS_FROM]

    # if sensor reply (0x0A is a bug workaround)
    if ( (msg.data[rcproto.POS_CMD] == rcproto.CMD_ANS_GET_SENS)
        or (msg.data[rcproto.POS_CMD] == 0x0A) ) :
        print('Got data for robot #{}'.format(robot_id))
        print('msg[ground] = {}, {}'.format(
            msg.data[rcproto.POS_DATA + 8],
            msg.data[rcproto.POS_DATA + 9]))

        print('msg[color] = {}'.format(
            msg.data[rcproto.POS_DATA + 10]))
        print('msg[ir] = {}, {}, {}, {}'.format(
            msg.data[rcproto.POS_DATA + 11],
            msg.data[rcproto.POS_DATA + 12],
            msg.data[rcproto.POS_DATA + 13],
            msg.data[rcproto.POS_DATA + 14]))


def init_system() :
    global cmd_pub
    global robots

    robots = []
    rospy.init_node("mobilization_ants")

    n_robots = rospy.get_param('~robots_num')

    # Входной топик
    rospy.Subscriber("/ardans_topic", ans, data_cb,
            queue_size= max(1,n_robots*7))

    # Выходной топик
    cmd_pub = rospy.Publisher("/actions_topic", action,
            queue_size=n_robots*7)

    rcx.set_cmd_pub(cmd_pub)

    # Создаем роботов (интерфейсные объекты)
    for id in range(1, n_robots+1) :
        # purely random agent
        # robots.append(Robot(id, agents.RandomAgent(range(4))))

        # roaming agent (FSM)
        robots.append(Robot(id, agents.RoamingFsmAgent(range(4))))

        # TODO: food searching agent (FSM)


def main():
    # Инициализация системы
    init_system()

    rate_hz = 10.
    print "Start main loop at {} Hz".format(rate_hz)
    rate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        for robot in robots:
            robot.update_sensors() # request new data
            robot.step() # continue doing current action if any

        rate.sleep()


if __name__ == '__main__':
    print("Ants Mobilization Foraging v1.0")

    main()

    print("Finished successfully")
