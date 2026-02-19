#!/usr/bin/env python
# coding: utf-8
"""
  simple_ants.py

  Demonstration of usage of FSMs created in a GUI for kvorum agent control.

  @author Rovbo Maxim
  @version 1.0
  @date 05.02.2019
"""
from __future__ import division

import os, sys, time
import copy
import roslib, rospy, random
import numpy

import rcproto
import rcproto_helper as rcx

from msg_rsaction.msg import action
from msg_ans.msg import ans

import agents.fsm_agents as agents

import logging
import yaml


class Robot(object) :
    """
    Robot interface abstraction.
    """

    # actions
    ACTION_DO_NOTHING = 0
    ACTION_STOP = 1
    ACTION_GO_FWD = 2
    ACTION_GO_BACK = 3
    ACTION_TURN_LEFT_10 = 4
    ACTION_TURN_RIGHT_10 = 5

    def __init__(self, id, controller) :
        self.id = id
        self.controller = controller

        self.observation = None

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

        # NOTE: currently not waiting for reply and using data that has been received before

    def update_observation(self, observation):
        self.observation = observation

    def step(self) :
        """ Perform an action.
        """
        action = self.controller.choose_action(self.observation)

        # do function from a dictionary of actions
        if action == self.ACTION_DO_NOTHING:
            pass
        elif action == self.ACTION_STOP:
            self.action_stop()
        elif action == self.ACTION_GO_FWD:
            self.action_go_fwd()
        elif action == self.ACTION_GO_BACK:
            self.action_go_back()
        elif action == self.ACTION_TURN_LEFT_10:
            self.action_turn_left_10()
        elif action == self.ACTION_TURN_RIGHT_10:
            self.action_turn_right_10()
        else:
            raise ValueError('Unrecognized action requested: {}'.format(action))

    def action_stop(self) :
        rcx.send_cmd(self.id, rcproto.CMD_STOP)

    def action_go_fwd(self) :
        rcx.send_cmd(self.id, rcproto.CMD_FWD)

    def action_go_back(self) :
        rcx.send_cmd(self.id, rcproto.CMD_BACK)

    def action_turn_left_10(self):
        rcx.send_cmd(self.id, rcproto.CMD_FAST_LEFT, 10)

    def action_turn_right_10(self):
        rcx.send_cmd(self.id, rcproto.CMD_FAST_RIGHT, 10)


class Observation(object):
    """
    Structured information about a robot's observation.
    """
    def __init__(self, ground_data, color_data, ir_data):
        self.ground = copy.copy(ground_data)
        self.color = copy.copy(color_data)
        self.ir = copy.copy(ir_data)

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.__dict == other.__dict__  # does element-wise ==
        else:
            return False

    def vectorize(self):
        """
        :return: A flat array of numbers representing the observation, that
        may omit some information for the sake of simplicity. Should not be
        used as a serialization or for comparison.
        """
        vectorized_observation = []
        vectorized_observation += self.ground
        vectorized_observation += self.color
        vectorized_observation += self.ir

        return vectorized_observation

    def __repr__(self):
        return str(self.vectorize())


def data_cb(msg) :
    """ Process received information from the robot or its
    simulated counterpart.
    """
    global robots

    robot_id = msg.data[rcproto.POS_FROM]

    # if sensor reply (0x0A is a bug workaround)
    if ( (msg.data[rcproto.POS_CMD] == rcproto.CMD_ANS_GET_SENS)
        or (msg.data[rcproto.POS_CMD] == 0x0A) ) :
        logger.debug('Got data for robot #{}'.format(robot_id))
        logger.debug('raw data: {}'.format(msg.data))
        logger.debug('msg[ground] = {}, {}'.format(
            msg.data[rcproto.POS_DATA + 8],
            msg.data[rcproto.POS_DATA + 9]))

        logger.debug('msg[color] = {}, {}, {}, {}'.format(
            msg.data[rcproto.POS_DATA + 10],
            msg.data[rcproto.POS_DATA + 11],
            msg.data[rcproto.POS_DATA + 12],
            msg.data[rcproto.POS_DATA + 13],
            ))

        # update observation in the correct robot
        robot = next(robot for robot in robots if robot.id == robot_id)
        observation = Observation(
                msg.data[rcproto.POS_DATA + 8 : rcproto.POS_DATA + 10],  # ground data in a slice
                msg.data[rcproto.POS_DATA + 10 : rcproto.POS_DATA + 14],  # color data in a slice
                msg.data[rcproto.POS_DATA + 15 : rcproto.POS_DATA + 16],  # ir data in a slice
                )
        # NOTE: ir data not used here and the agent does not have such sensors defined
        # it is just an example of how to get this data
        robot.update_observation(observation)


def setup_logging():
    global logger

    log_config = rospy.get_param('~log_config')

    if os.path.exists(log_config):
        with open(log_config, 'rt') as f:
            config = yaml.safe_load(f.read())
    else:
        raise FileNotFoundError('No logging configuration file found with name: {}'.format(
            log_config
            ))

    logging.config.dictConfig(config)
    logger = logging.getLogger(__name__)


def init_system() :
    global cmd_pub
    global robots

    robots = []
    rospy.init_node("fsm_ants")

    n_robots = rospy.get_param('~robots_num')

    setup_logging()

    logger.info("Simple FSM demonstration v1.0")

    # Входной топик
    rospy.Subscriber("/ardans_topic", ans, data_cb,
            queue_size= max(1,n_robots*7))

    # Выходной топик
    cmd_pub = rospy.Publisher("/actions_topic", action,
            queue_size=n_robots*7)

    rcx.set_cmd_pub(cmd_pub)

    # Создаем роботов (интерфейсные объекты)
    for id in range(1, n_robots+1) :
        # create a roaming agent (FSM)
        robots.append(Robot(id, agents.RoamingFsmAgent(Robot, obstacle_threshold=20, trace=True)))


def main_timer_cb(event):
    for robot in robots:
        robot.update_sensors() # request new data
        robot.step() # continue doing current action if any


def main():
    # Инициализация системы
    init_system()

    rate_hz = 10.
    logger.info("Start main loop at {} Hz".format(rate_hz))

    rospy.Timer(rospy.Duration(1 / rate_hz), main_timer_cb, oneshot=False)
    rospy.spin()


if __name__ == '__main__':
    main()
