#!/usr/bin/env python3
# coding: utf-8

"""

Мозги

Version 1.06

06.03.2021, 03.07.2021, 11.02.2023
LP 18.02.2024

"""

import math

from gmodctl import mobot_models as models
from gmodctl import mathfish

# Цель -- ArUco-маркеры со следующими кодами:
GoalTargets = [9,10,11,12,13,14,15]

class TRobotState(object):
    def __init__(self):
        self.goalpos = None
        self.cmd = None
        self.checkreflex = [True, True]
        self.reflexlevel = None

RobotState = TRobotState()

def Init():
    pass

def GetGoalPos(Omega):
    for e in Omega:
        pos = e[0]
        tip = e[1]
        a = math.atan2(pos[1], pos[0])
        r = math.hypot(pos[1], pos[0])
        if tip in GoalTargets:
            return (-a, r, tip)
    return None

Robot = models.TAgent()

def print_log(): pass

#
# Преобразование пары (линейная_скорость, угол_ориентации)
# в пару (линейная_скорость, угловая_скорость)
# Вход: vlin, vang - угол ориентации в градусах
# Выход: линейная скорость, угловая скорость (рад.)
# Пропорциональная зависимость: чем больше угол ориентации, тем меньше линейная скорость
#
def EvalVLinAng(vlin, vang):
    def pmap(a, a0, a1, v0, v1): return (a-a0)*(v1-v0)/(a1-a0) + v0

    if abs(vang)>90: vang = 90*mathfish.sign(vang)
    v = pmap(abs(vang), 0, 90, vlin, 0.1)*mathfish.sign(vlin)

    alim = 3
    if abs(vang)>alim: vang = mathfish.sign(vang)*alim
    vang = math.radians(vang)
    return v, vang

def MakeStep(Sensors, Omega):

    # Рефлексы
    RDIST0 = 0.5
    RDIST1 = 1.0
    # Level-0
    RobotState.reflexlevel = None
    if(RobotState.checkreflex[0]):
        if Sensors['usonic_fwd_center'] < RDIST0*2/3:
            RobotState.reflexlevel = 0
            RobotState.cmd = ('CMD_GO_BACK_FAST', (0,0))
        if Sensors['usonic_fwd_left'] < RDIST0:
            RobotState.reflexlevel = 0
            RobotState.cmd = ('CMD_GO_RIGHT_FAST', (0,0))
        if Sensors['usonic_fwd_right'] < RDIST0:
            RobotState.reflexlevel = 0
            RobotState.cmd = ('CMD_GO_LEFT_FAST', (0,0))
        if not (RobotState.reflexlevel is None): return RobotState.cmd

    # Level-1
    if(RobotState.checkreflex[1]):
        if Sensors['usonic_fwd_center'] < RDIST1*2/3:
            RobotState.reflexlevel = 1
            RobotState.cmd = ('CMD_GO_LEFT_SLOW', (0,0))
        if Sensors['usonic_fwd_left'] < RDIST1:
            RobotState.reflexlevel = 1
            RobotState.cmd = ('CMD_GO_RIGHT_SLOW', (0,0))
        if Sensors['usonic_fwd_right'] < RDIST1:
            RobotState.reflexlevel = 1
            RobotState.cmd = ('CMD_GO_LEFT_SLOW', (0,0))
        if not (RobotState.reflexlevel is None): return RobotState.cmd

    # Штатное управление
    vctl = models.FObstAlong(Robot, Omega, add_angle=90, debug=True)
    vlin = vctl[0]
    vang = vctl[1][0]

    # Формируем что-то более адекватное
    vlin, vang = EvalVLinAng(vlin, vang)

    RobotState.cmd = ('CMD_TWIST', (vlin, vang))
    return RobotState.cmd

    #
    #
    #
    gp = GetGoalPos(Omega)
    Sensors['goal'] = gp
    if gp:
        RobotState.reflexlevel = None
        RobotState.goalpos = gp
        RobotState.checkreflex = [False, False]
        RobotState.cmd = ('CMD_GO_GOAL', gp)
        return RobotState.cmd

    RobotState.reflexlevel = None
    RobotState.goalpos = None
    RobotState.checkreflex = [True, True]
    RobotState.cmd = ('CMD_GO_FWD_FAST', (0,0))

    return RobotState.cmd
