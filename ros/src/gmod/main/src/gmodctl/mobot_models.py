#!/usr/bin/env python3
# coding: utf-8
'''
  Поведенческие процедуры
  V 1.04
  26.03.2022, 15.04.2022, 29.12.2023
  LP 15.02.2024
'''
import math
import numpy as np

from gmodctl import mathfish

def print_log(): pass

class TAgent(object):
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.angle = [0,0,0]
        self.C = {'size': 1, 'speed':1}

################################################################################
#
# Ядро процедур движения вдоль ориентиров с использованием метода потенциальных полей
#
# rdim_attr - размерность закона притяжения
# rdim_sep  - размерность закона отталкивания (чем больше, тем "точнее" идет вдоль пятен/препятствий
# coeff_d   - коэффициент, определяющий границу сил отталкивания/притяжения (чем меньше, тем ближе)
# В итоге: rdim - точность, coeff_d - характер переходного процесса 
#
# Важно: возвращаются линейная скорость и угол ориентации yaw
#
# Параметры:
#
################################################################################

def BaseSpotMovement(agent, Omega, normspeed, add_angle, rdim_attr, rdim_sep, coeff_d, debug):
    spots = Omega

    # Никого не видим. Крутимся на месте
    if len(spots)==0:
        if debug: print_log('-')
        return normspeed/2, [agent.angle[0] - mathfish.sign(add_angle)*2, 0, 0]

    k_attr, k_sep = 1.0, 1.0

    # dist_min = math.hypot(fagent.ENV['WX'][1], fagent.ENV['WY'][1], fagent.ENV['WZ'][1])
    dist_min = None
    dist_force = agent.C['size']*coeff_d

    attr_speed, attr_angle = 0, [0,0,0]
    sep_speed, sep_angle = 0, [0,0,0]
    res_speed, res_angle = 0, [0,0,0]
    # Цикл по всем объектам из зоны видимости
    for s in spots:
        s_x = s[0][0]
        s_y = s[0][1]
        s_z = 0
        A = np.array([s_x - agent.x, s_y - agent.y, s_z - agent.z])
        dist = np.linalg.norm(A)/dist_force

        if (dist_min is None) or (dist_min>dist): dist_min = dist

        a_attr = mathfish.Dec2SphCoord(A)

        # Учет направления
        # Косинусный закон: значение имеют те ориентиры, которые впереди
        k_cos = math.cos(math.radians(agent.angle[0] - a_attr[0]))
        if(k_cos<0): k_cos = 0

        # Учет фактора близости (видимость)
        k_near = 1 if dist>=0.5 else 0

        # Притяжение
        if dist>1:
            # Можно использовать оба эти закона. Несущественно
            # force = math.log(dist)
            force = k_cos*k_near*(dist**rdim_attr)
            sp = k_attr*force
            attr_speed, attr_angle = mathfish.SumVect(attr_speed, attr_angle, sp, a_attr)
        else:
        # Отталкивание
            a_sep = [a_attr[0]+180, a_attr[1]+180, a_attr[2]]
            #force = -math.log(dist)
            force = k_cos*k_near*(dist**rdim_sep)
            sp = k_sep*force
            sep_speed, sep_angle = mathfish.SumVect(sep_speed, sep_angle, sp, a_sep)

    _, res_angle = mathfish.SumVect(sep_speed, sep_angle, attr_speed, attr_angle)

    # Поворот, чтобы иди боком
    d = dist_force*(dist_min-1)
    if d<0: d=0
    rota0 = add_angle*(math.exp(-d))
    # sic
    yaw = res_angle[0]+rota0

    # Нормируем угол в диапазоне -180 - +180
    # Это для наглядности
    yaw = yaw % 360
    if yaw>180: yaw = yaw - 360
    if yaw<-180: yaw = 360+yaw

    res_angle = [yaw, res_angle[1], res_angle[2]]

    # Скорость постоянная
    res_speed = normspeed

    if debug:
        print_log('=> sep=({:4.2f}, {:4.2f}), attr=({:4.2f}, {:4.2f}), CTL=(v={:<4.2f}, a={:<4.2f}) dist={:4.2f} (dist_force={:4.2f}) da={:4.2f} k_cos={:4.2f} k_near={:4.2f}'
             .format(sep_speed, sep_angle[0], attr_speed, attr_angle[0], res_speed, res_angle[0], dist_min, dist_force, rota0, k_cos, k_near))

    return res_speed, res_angle

################################################################################
#
# Движение вдоль препятствий
# Желательно, чтобы препятствия образовывали выпуклые области
#
################################################################################
def FObstAlong(agent, Omega, add_angle=90, debug=False):
    rdim_attr = -1  # -2
    rdim_sep = -2   # -2 Чем больше, тем "точнее" идет вдоль ориентиров препятствий
    coeff_d = 2     # 4 Чем меньше, тем ближе
    normspeed = agent.C["speed"]/2
    return BaseSpotMovement(agent, Omega, normspeed, add_angle, rdim_attr, rdim_sep, coeff_d, debug)

################################################################################
#
# Движение по пятнам-ориентирам
# "Пятно" - это буй
#
################################################################################
def FSpotMovement(agent, Omega, add_angle=90, debug=False):
    rdim_attr = -1   # -2
    rdim_sep = -2    # Чем больше, тем "точнее" идет вдоль пятен
    coeff_d = 2      # Чем меньше, тем ближе
    normspeed = agent.C["speed"]/2
    return BaseSpotMovement(agent, Omega, normspeed, add_angle, rdim_attr, rdim_sep, coeff_d, debug)

################################################################################
#
#
#
################################################################################
if __name__ == "__main__" :
    pass
