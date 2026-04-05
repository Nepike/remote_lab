#!/usr/bin/env python3
# coding: utf-8

"""
  Описание среды для визуализации
  Author: Valery Karpov

  06.02.2015, 18.07.2016
  Version 3.02
  LP 03.03.2024

"""
import sys
import math
from kvorum2 import gdic
from kvorum2.tshapes import WPen
from kvorum2.env import TEnv

################################################################################
#
#
#
################################################################################
# http://www.discoveryplayground.com/computer-programming-for-kids/rgb-colors/
# Номера цветов и их изображение
CCOLOR = { 0: "black",

           1: "Blue",
           2: "Light Sky Blue",
           3: "Turquoise",
           4: "Cyan",

           5: "Medium Aquamarine",
           6: "Aquamarine",
           7: "Spring Green",
           8: "Chartreuse",

           9: "Light Salmon",
          10: "Orange",
          11: "Tomato",
          12: "Dark Salmon"}

class TGrEnv(TEnv):

    global WPen

    # Размер окна
    MAX_SCR_X = 500
    MAX_SCR_Y = 500
    OFFS_X = 20
    OFFS_Y = 20

    def __init__(self, filename):
        TEnv.__init__(self, filename)

    # Вычисление экранных координат
    def Coord2Scr(self, x, y):
        minx = -self.MAX_SCR_X/2
        maxx =  self.MAX_SCR_X/2
        miny = -self.MAX_SCR_Y/2
        maxy =  self.MAX_SCR_Y/2
        xscr = (x*(maxx-minx)/self.DIM_X) + minx+self.OFFS_X
        yscr = (y*(maxy-miny)/self.DIM_Y) + miny+self.OFFS_Y
        return int(xscr), int(yscr)

    #
    # Рисуем ячейку
    #
    def DrawCell(self, x, y):
        gridsize = self.MAX_SCR_X/self.DIM_X
        if(gridsize<1): gridsize = 1
        needBorder = (gridsize > 5)
        (xscr, yscr) = self.Coord2Scr(x, y)
        WPen.DrawRect(xscr, yscr, xscr+gridsize, yscr+gridsize, "white", "white")

        # Рисуем в обратном порядке, чтоб LEVEL_GROUND не перекрывал бы никого
        #for level in range(gdic.LEVEL_LIGHT, gdic.LEVEL_GROUND+1):
        for level in range(gdic.LEVEL_GROUND, gdic.LEVEL_LIGHT-1, -1):
            if(level==gdic.LEVEL_TEXT): continue  # С текстом будем разбираться отдельно
            val = self.Field[y][x][level]
            if(val != 0):
                ccolor = gdic.LevelColor[level]
                # Отдельно определяемся с цветом для слоя gdic.LEVEL_COLOR
                if(level==gdic.LEVEL_COLOR):
                    n = self.Field[y][x][level];
                    if(n<=12):
                        ccolor = CCOLOR[n]
                    else:
                        R, G, B = n, n, n
                        ccolor = '#%02X%02X%02X' % (R, G, B)
                if(needBorder):
                    outlinecolor = "white"
                else:
                    outlinecolor = ccolor
                WPen.DrawRect(xscr, yscr, xscr+gridsize, yscr+gridsize, ccolor, outlinecolor)

    #
    # Рисуем все объекты
    #
    def DrawField(self):
        x0, y0 = -self.MAX_SCR_X/2+self.OFFS_X, -self.MAX_SCR_Y/2+self.OFFS_Y
        x1, y1 =  self.MAX_SCR_X/2+self.OFFS_X,  self.MAX_SCR_Y/2+self.OFFS_Y
        # Рисуем рамку
        WPen.DrawRect(x0-1, y0-1, x1+1, y1+1, "white","black")
        # Рисуем объекты
        for y in range(0, self.DIM_Y):
            for x in range(0, self.DIM_X):
                self.DrawCell(x, y)
        # Рисуем текст отдельно
        for y in range(0, self.DIM_Y):
            for x in range(0, self.DIM_X):
                val = self.Field[y][x][gdic.LEVEL_TEXT]
                if(val != 0):
                    (xscr, yscr) = self.Coord2Scr(x, y)
                    WPen.Write(xscr, yscr, val)
