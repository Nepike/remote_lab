#!/usr/bin/env python3
# coding: utf-8

"""
  Основные геометрические функции
  Author: Valery Karpov
  (виртуальное рисование)
"""
import math

################################################################################
#
################################################################################

HH = { 1: ( 1,  1), 2: (-1,  1), 3: (-1, -1), 4: ( 1, -1) }

BRDSIN = []
BRDCOS = []
def BradisInit():
    print("- BradisInit")
    for a in range(0, 360+1):
        #g = math.pi / 180.0 * a
        g = math.radians(a)
        vsin = math.sin(g)
        vcos = math.cos(g)
        BRDSIN.append(vsin)
        BRDCOS.append(vcos)

def FastSin(a):
    """
    a = int(a)
    a = a % 360
    if(a<0):
      a += 360
    return BRDSIN[a]
    """
    g = math.radians(a)
    return math.sin(g)

def FastCos(a):
    """
    a = int(a)
    a = a % 360
    if(a<0):
      a += 360
    return BRDCOS[a]
    """
    g = math.radians(a)
    return math.cos(g)

def __getqw(a):
    a = a % 360
    if(a <= 90): return 1
    elif(a <= 180): return 2
    elif(a <= 270): return 3
    else: return 4

################################################################################
#
# BresArc1 Базовый алгоритм построения дуги окружности
#
# Возвращает 2 значения: (res, val)
#   res - признак того, что все дорисовано до конца и plotfunction отработала полностью
#   val - значение, возвращенное функцией plotfunction
#
#  Свойства plotfunction:
#    Аргументы: координаты x, y
#    Возвращает 2 значения: (res, val)
#      res - True - обнаружена искомая точка
#            False - признак того, что ничего интересного не обнаружено
#      val - значение среды в исследуемой точке
#
################################################################################

def BresArc1(x0, y0, a1, a2, R, plotfunction, H):

    h1, h2 = H
    # стартовый и конечный углы
    x = int(R * FastCos(a2))
    y = int(R * FastSin(a2))

    d = int((x + 1) * (x + 1) + (y - 1) * (y - 1) - R * R)
    #lim = int(R * math.sin(g1))
    lim = int(R * FastSin(a1))
    while(y >= lim):
        # Вывод (чтение) точки
        res, val = plotfunction(h1*(x) + (x0), h2*(y) + (y0))
        if(res): return (res, val)
        if (d < 0):
            dd = 2 * d + 2 * y - 1
            if (dd <= 0):
                x+=1
                d += (2 * x + 1)
            else:
                x+=1
                y-=1
                d += (2 * x - 2 * y + 2);
        if (d > 0):
            dd = 2 * d - 2 * x - 1;
            if (dd <= 0):
                x+=1
                y-=1
                d += (2 * x - 2 * y + 2)
            else:
                y-=1
                d += (-2 * y + 1)
        if (d == 0):
            x+=1
            y-=1
            d += (2 * x - 2 * y + 2)
    return (False, None)

################################################################################
#
#  BresArc Основной алгоритм построения дуги окружности
#  Для рисования окружности надо задать углы a1=0, a2=360
#
#  Возвращает 2 значеня, взятые из BresArc1
#
################################################################################
def BresArc(x0, y0, a1, a2, R, plotfunction):

    iscirc = (a1==0 and a2==360)

    # Определяем квадранты для углов
    a1 = a1 % 360
    a2 = a2 % 360
    Q1 = __getqw(a1)
    Q2 = __getqw(a2)

    #################################################
    # a1<=a2
    #################################################
    if(a1<=a2 and not iscirc):
        if(Q1==1):
            ### 1-1
            if(Q2==1):
                e, val = BresArc1(x0, y0, a1, a2, R, plotfunction, HH[1])
                if(e): return (e, val)
            ### 1-2
            if(Q2==2):
                e, val = BresArc1(x0, y0,     a1, 90, R, plotfunction, HH[1])
                if(e): return (e, val)
                e, val = BresArc1(x0, y0, 180-a2, 90, R, plotfunction, HH[2])
                if(e): return (e, val)
            ### 1-3
            if(Q2==3):
                e, val = BresArc1(x0, y0, a1,     90, R, plotfunction, HH[1])
                if(e): return (e, val)
                e, val = BresArc1(x0, y0,  0,     90, R, plotfunction, HH[2])
                if(e): return (e, val)
                e, val = BresArc1(x0, y0,  0, a2-180, R, plotfunction, HH[3])
                if(e): return (e, val)
            ### 1-4
            if(Q2==4):
                e, val = BresArc1(x0, y0,     a1, 90, R, plotfunction, HH[1])
                if(e): return (e, val)
                e, val = BresArc1(x0, y0,      0, 90, R, plotfunction, HH[2])
                if(e): return (e, val)
                e, val = BresArc1(x0, y0,      0, 90, R, plotfunction, HH[3])
                if(e): return (e, val)
                e, val = BresArc1(x0, y0, 360-a2, 90, R, plotfunction, HH[4])
                if(e): return (e, val)
        if(Q1==2):
            ### 2-2
            if(Q2==2):
                e, val = BresArc1(x0, y0, 180-a2, 180-a1, R, plotfunction, HH[2])
                if(e): return (e, val)
            ### 2-3
            if(Q2==3):
                e, val = BresArc1(x0, y0, 0, 180-a1, R, plotfunction, HH[2])
                if(e): return (e, val)
                e, val = BresArc1(x0, y0, 0, a2-180, R, plotfunction, HH[3])
                if(e): return (e, val)
            ### 2-4
            if(Q2==4):
                e, val = BresArc1(x0, y0, 0,  180-a1, R, plotfunction, HH[2])
                if(e): return (e, val)
                e, val = BresArc1(x0, y0, 0,      90, R, plotfunction, HH[3])
                if(e): return (e, val)
                e, val = BresArc1(x0, y0, 360-a2, 90, R, plotfunction, HH[4])
                if(e): return (e, val)
        if(Q1==3):
            ### 3-3
            if(Q2==3):
                e, val = BresArc1(x0, y0, a1-180, a2-180, R, plotfunction, HH[3])
                if(e): return (e, val)
            ### 3-4
            if(Q2==4):
                e, val = BresArc1(x0, y0, a1-180, 90, R, plotfunction, HH[3])
                if(e): return (e, val)
                e, val = BresArc1(x0, y0, 360-a2, 90, R, plotfunction, HH[4])
                if(e): return (e, val)
        if(Q1==4):
            ### 4-4
            if(Q2==4):
                e, val = BresArc1(x0, y0, 360-a2, 360-a1, R, plotfunction, HH[4])
                if(e): return (e, val)

    #################################################
    # a1>a2
    #################################################
    if(a1>a2 or iscirc):
        if(Q2==1):
            e, val = BresArc1(x0, y0,  0, a2, R, plotfunction, HH[1])
            if(e): return (e, val)
            ### 1-1
            if(Q1==1):
                e, val = BresArc1(x0, y0, a1, 90, R, plotfunction, HH[1])
                if(e): return (e, val)
                e, val = BresArc1(x0, y0,  0, 90, R, plotfunction, HH[2])
                if(e): return (e, val)
                e, val = BresArc1(x0, y0,  0, 90, R, plotfunction, HH[3])
                if(e): return (e, val)
                e, val = BresArc1(x0, y0,  0, 90, R, plotfunction, HH[4])
                if(e): return (e, val)
            ### 2-1
            if(Q1==2):
                e, val = BresArc1(x0, y0,  0, 180-a1, R, plotfunction, HH[2])
                if(e): return (e, val)
                e, val = BresArc1(x0, y0,  0,     90, R, plotfunction, HH[3])
                if(e): return (e, val)
                e, val = BresArc1(x0, y0,  0,     90, R, plotfunction, HH[4])
                if(e): return (e, val)
            ### 3-1
            if(Q1==3):
                e, val = BresArc1(x0, y0,  a1-180, 90, R, plotfunction, HH[3])
                if(e): return (e, val)
                e, val = BresArc1(x0, y0,       0, 90, R, plotfunction, HH[4])
                if(e): return (e, val)
            ### 4-1
            if(Q1==4):
                e, val = BresArc1(x0, y0,  0,  360-a1, R, plotfunction, HH[4])
                if(e): return (e, val)
        if(Q2==2):
            e, val = BresArc1(x0, y0, 180-a2, 90, R, plotfunction, HH[2])
            if(e): return (e, val)
            e, val = BresArc1(x0, y0,      0, 90, R, plotfunction, HH[1])
            if(e): return (e, val)
            ### 2-2
            if(Q1==2):
                e, val = BresArc1(x0, y0, 0, 180-a1, R, plotfunction, HH[2])
                if(e): return (e, val)
                e, val = BresArc1(x0, y0, 0,     90, R, plotfunction, HH[3])
                if(e): return (e, val)
                e, val = BresArc1(x0, y0, 0,     90, R, plotfunction, HH[4])
                if(e): return (e, val)
            ### 3-2
            if(Q1==3):
                e, val = BresArc1(x0, y0, a1-180, 90, R, plotfunction, HH[3])
                if(e): return (e, val)
                e, val = BresArc1(x0, y0,      0, 90, R, plotfunction, HH[4])
                if(e): return (e, val)
            ### 4-2
            if(Q1==4):
                e, val = BresArc1(x0, y0,      0, 360-a1, R, plotfunction, HH[4])
                if(e): return (e, val)
        if(Q2==3):
            e, val = BresArc1(x0, y0, 0, a2-180, R, plotfunction, HH[3])
            if(e): return (e, val)
            e, val = BresArc1(x0, y0, 0,     90, R, plotfunction, HH[1])
            if(e): return (e, val)
            e, val = BresArc1(x0, y0, 0,     90, R, plotfunction, HH[2])
            if(e): return (e, val)
            ### 3-3
            if(Q1==3):
                e, val = BresArc1(x0, y0, 0,     90, R, plotfunction, HH[4])
                if(e): return (e, val)
                e, val = BresArc1(x0, y0, a1-180,90, R, plotfunction, HH[3])
                if(e): return (e, val)
            ### 4-3
            if(Q1==4):
                e, val = BresArc1(x0, y0, 0, 360-a1, R, plotfunction, HH[4])
                if(e): return (e, val)
        if(Q2==4):
            ### 4-4
            if(Q1==4):
                e, val = BresArc1(x0, y0,      0,     90, R, plotfunction, HH[1])
                if(e): return (e, val)
                e, val = BresArc1(x0, y0,      0,     90, R, plotfunction, HH[2])
                if(e): return (e, val)
                e, val = BresArc1(x0, y0,      0,     90, R, plotfunction, HH[3])
                if(e): return (e, val)
                e, val = BresArc1(x0, y0, 360-a2,     90, R, plotfunction, HH[4])
                if(e): return (e, val)
                e, val = BresArc1(x0, y0,      0, 360-a1, R, plotfunction, HH[4])
                if(e): return (e, val)

    return (False, None)

################################################################################
#
# Заполненный сектор
#
################################################################################
def BresFillArc(x0, y0, a1, a2, R, plotfunction):
    for r in range(1, R+1):
        e, val = BresArc(x0, y0, a1, a2, r, plotfunction)
        if(e): return (e, (r, val))
    return (False, None)

################################################################################
#
# Круг
#
################################################################################
def BresFillCircle(x0, y0, R, plotfunction):
    e, val = BresFillArc(x0, y0, 0, 360, R, plotfunction)
    return (e, val)

################################################################################
#
# Линия-1
#
################################################################################
def BresLineP2P(x0, y0, x1, y1, plotfunction):
    # Проверяем рост отрезка по оси X и Y
    steep = abs(y1 - y0) > abs(x1 - x0)
    # Отражаем линию по диагонали, если угол наклона слишком большой
    if(steep):
        # Swap
        x0, y0 = y0, x0
        x1, y1 = y1, x1

    # Если линия растёт не слева направо, то меняем начало и конец отрезка местами
    if(x0 > x1):
        # Swap
        x0, x1 = x1, x0
        y0, y1 = y1, y0
    dx = x1 - x0
    dy = abs(y1 - y0)
    error = dx / 2 # Здесь используется оптимизация с умножением на dx, чтобы избавиться от лишних дробей

    # Выбираем направление роста координаты y
    if(y0<y1): ystep = 1
    else: ystep = -1

    y = y0
    for x in range(x0, x1+1):
        R = x-x0 # ????
        if(steep):
            r, val = plotfunction(y, x)
        else:
            r, val = plotfunction(x, y)
        if(r): return (True, (R, val))

        error -= dy
        if (error < 0):
            y += ystep
            error += dx

    return (False, None)

################################################################################
#
# Линия-2
# a - угол
# R - длина отрезка
#
################################################################################
def BresLine(x0, y0, a, R, plotfunction):
    res = BresLineC(x0, y0, a, R, plotfunction)
    return res

def BresLineA(x0, y0, a, R, plotfunction):
    x1 = int(x0 + R * FastCos(a))
    y1 = int(y0 + R * FastSin(a))
    res = BresLineP2P(x0, y0, x1, y1, plotfunction)
    return res

def BresLineB(x0, y0, a, R, plotfunction):
    x1 = int(x0 + R * FastCos(a))
    y1 = int(y0 + R * FastSin(a))
    steep = abs(y1 - y0) > abs(x1 - x0)

    # Вертикаль
    if(x0==x1):
        if(y0<y1):
            stepy = 1
        else:
            stepy = -1
        y = y0
        x = x0
        while(y!=y1):
            y += stepy
            r, val = plotfunction(int(x), int(y))
            dist = int(math.hypot(x0-x, y0-y))
            if(r): return (True, (dist, val))
        return (False, None)

    # Все хорошо
    k = float(y0-y1)/float(x0-x1)
    b = y1-k*x1

    x = x0
    y = y0

    if(steep):
        if(y0<y1):
            stepy = 1
        else:
            stepy = -1
        while(y!=y1):
            y += stepy
            x = (y-b)/k
            r, val = plotfunction(int(x), int(y))
            if(r): 
                dist = int(math.hypot(x0-x, y0-y))
                return (True, (dist, val))
    else:
        if(x0<x1):
            stepx = 1
        else:
            stepx = -1
        while(x!=x1):
            x += stepx
            y = k*x+b
            r, val = plotfunction(int(x), int(y))
            if(r):
                dist = int(math.hypot(x0-x, y0-y))
                return (True, (dist, val))

    return (False, None)

def BresLineC(x0, y0, a, R, plotfunction):
    x1 = int(x0 + R * FastCos(a))
    y1 = int(y0 + R * FastSin(a))

    steep = abs(y1 - y0) > abs(x1 - x0)

    # Вертикаль
    if(x0==x1):
        if(y0<y1):
            stepy = 1
        else:
            stepy = -1
        y = y0
        x = x0
        while(y!=y1):
            y += stepy
            r, val = plotfunction(int(x), int(y))
            if(r): 
                dist = int(math.hypot(x0-x, y0-y))
                return (True, (dist, val))
        return (False, None)

    x = x0
    y = y0
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)

    if(steep):
        # Движение по y
        if(y0<y1):
            stepy = 1
        else:
            stepy = -1

        # Выбираем направление роста координаты x
        if(x0<x1): stepx = 1
        elif(x0>x1): stepx = -1
        else: stepx = 0

        error = dy / 2 # Здесь используется оптимизация с умножением на dx, чтобы избавиться от лишних дробей

        while(y!=y1):
            y += stepy

            error -= dx
            if (error < 0):
                x += stepx
                error += dy

            r, val = plotfunction(int(x), int(y))
            if(r): 
                dist = int(math.hypot(x0-x, y0-y))
                return (True, (dist, val))
    else:
        # Движение по x
        if(x0<x1):
            stepx = 1
        else:
            stepx = -1

        # Выбираем направление роста координаты y
        if(y0<y1): stepy = 1
        elif(y0>y1): stepy = -1
        else: stepy = 0

        error = dx / 2 # Здесь используется оптимизация с умножением на dx, чтобы избавиться от лишних дробей

        while(x!=x1):

            x += stepx

            error -= dy
            if (error < 0):
                y += stepy
                error += dx
            r, val = plotfunction(int(x), int(y))
            if(r): 
                dist = int(math.hypot(x0-x, y0-y))
                return (True, (dist, val))

    return (False, None)

################################################################################
# Init Module
################################################################################
BradisInit()
