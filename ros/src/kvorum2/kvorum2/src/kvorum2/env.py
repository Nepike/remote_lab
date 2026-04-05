#!/usr/bin/env python3
# coding: utf-8

"""
  Описание модельной среды
  Author: Valery Karpov

  06.02.2015, 22.03.2016, 23.03.2024
  Version 3.02
  LP 15.08.2024

"""
import sys
import math

from kvorum2 import gdic
from kvorum2 import geometry

################################################################################
#
# Класс "Среда"
#
################################################################################
# Эти глобалы нужны функции SFPlot(x, y), которая вызывается из функции SetRoundField
SFlevel = 0
SFvalue = 0
SFenv = None

class TEnv:
    def __init__(self, filename):
        # Собственно поле - среда
        self.Field = []
        # Физический размер поля
        self.DIM_X = 1
        self.DIM_Y = 1

        self.UseTorusRegime = True   # Флаг режима топологии тора

        self.Rate = 50               # Частота, Гц

        self.ReadCtlFile(filename)

    #
    # Исполнение файла конфигурации
    #
    def ReadCtlFile(self, filename):
        print("Read (execute) ctlfile", filename, "...")
        exec (open(filename).read())
        for y in range(0, self.DIM_Y):
            row = []
            for x in range(0, self.DIM_X):
                row.append([0,0,0,0,0,0])
            self.Field.append(row)
            
        # Если топология - не тор, то обрамляем поле препятствиями
        if(not self.UseTorusRegime):
            for y in range(0, self.DIM_Y):
                self.Field[y][0][gdic.LEVEL_GROUND] = 1
                self.Field[y][self.DIM_X-1][gdic.LEVEL_GROUND] = 1
            for x in range(0, self.DIM_X):
                self.Field[0][x][gdic.LEVEL_GROUND] = 1
                self.Field[self.DIM_Y-1][x][gdic.LEVEL_GROUND] = 1
        print("ReadCtlFile:", self.DIM_Y, self.DIM_X)
        print("Done")

    # Установить значение поля в точке
    def SetFieldVal(self, x, y, level, value):
        x = int(x)
        y = int(y)
        if(x<0): x = 0
        if(y<0): y = 0
        if(x>=self.DIM_X): x = self.DIM_X-1
        if(y>=self.DIM_Y): y = self.DIM_Y-1
        self.Field[y][x][level] = value

    # Заполнить поле по кругу
    def SetRoundField(self, x, y, R, level, value):
        global SFlevel, SFvalue, SFenv
        SFlevel = level
        SFvalue = value
        SFenv = self
        geometry.BresFillArc(x, y, 0, 360, R, SFPlot)

    def SetLineField(self, x, y, a, R, level, value):
        global SFlevel, SFvalue, SFenv
        SFlevel = level
        SFvalue = value
        SFenv = self
        geometry.BresLine(int(x), int(y), int(a), int(R), SFPlot)

    # Заполнить прямоугольную область поля
    def SetRectangleField(self, x1, y1, x2, y2, level, value):
        if(x1>x2): x1, x2 = x2, x1
        if(y1>y2): y1, y2 = y2, y1
        for i in range(x1, x2+1):
            for j in range(y1, y2+1):
                (x, y) = self.normalizate_xy(i, j)
                self.SetFieldVal(x, y, level, value)

    def Text(self, x, y, text):
        self.SetFieldVal(int(x), int(y), gdic.LEVEL_TEXT, text)

    # Вычисление "нормальных" координат
    def normalizate_xy(self, x, y):
        if self.UseTorusRegime:
            if(x<0): x = x % self.DIM_X
            if(x>=self.DIM_X): x = x % self.DIM_X
            if(y<0): y = y % self.DIM_Y
            if(y>=self.DIM_Y): y = y % self.DIM_Y
        else:
            # NOTE: possibly incorrect processing for sensor data
            # and BresArc usage as it ignores incline
            if(x<0): x = 0
            if(x>=self.DIM_X): x = self.DIM_X-1
            if(y<0): y = 0
            if(y>=self.DIM_Y): y = self.DIM_Y-1
        return (x, y)

    # Получить объект по его координатам
    def GetObject(self, x, y):
        (x1, y1) = self.normalizate_xy(x, y)
        x1 = int(x1)
        y1 = int(y1)

        try :
            obj = self.Field[y1][x1]
        except IndexError :
            print('Index {},{} is unavailable'.format(x1,y1))
            print('Before {},{} is unavailable'.format(x,y))
            raise

        return obj

#
# Вспомогательная функция заполнения поля
#
def SFPlot(x, y):
    global SFlevel, SFvalue, SFenv
    x, y = SFenv.normalizate_xy(x, y)
    SFenv.SetFieldVal(x, y, SFlevel, SFvalue)
    return (False, None)
