#!/usr/bin/env python3
# coding: utf-8

"""
  Viz Agent module
  Author: Valery Karpov

  06.02.2015, 22.11.2017, 11.02.2024
  Version 2.07
  LP 15.08.2024

"""

import sys
import random
import math

from kvorum2 import gdic
from kvorum2.tshapes import TPen, WPen
from kvorum2 import geometry
from kvorum2.agent import TAgent, TSensor

################################################################################
#
# Агент
#
################################################################################

TRACECOLOR = ["Black",
              "Red",
              "Yellow",
              "Blue",
              "Cyan",
              "Spring Green",
              "Chartreuse",
              "Light Salmon",
              "Orange",
              "Tomato",
              "Dark Salmon"]

class TGrAgent(TAgent):
    global Pen, TRACECOLOR
    def __init__(self, cid, cpos, cshape, cenv, show_id = False, csize = 0):
        # csize - размер объекта: 0 - по умолчанию, -1 - по размеру сетки, иначе - по заданному значению
        TAgent.__init__(self, cid, cpos, cshape, cenv, show_id, csize)

        self.FirstDraw = True # Флаг того, что рисование только началось (для режима рисования следа traceOn)
        if show_id: aid = cid
        else: aid = None

        self.Img = TPen(self.shape, usrid = aid)

        # Разбираемся с цветом
        bordercolor = "green" #red
        self.Img.Color(bordercolor, "black")

        self.Img.PenSize(5) # Это для режима рисования следа
        self.Img.speed(0)

        # Разбираемся с размером
        if(csize>0):
            self.Img.ShapeSize(csize)
        if(csize==-1):
            gridsize = cenv.MAX_SCR_X/cenv.DIM_X
            if(gridsize<1): gridsize = 1
            self.Img.ShapeSize(gridsize)

        self.pred_xscr = cpos[0]
        self.pred_yscr = cpos[1]

    #
    # Рисование объекта
    #
    def Draw(self):
        xscr, yscr = self.env.Coord2Scr(self.GetX(), self.GetY())
        self.Img.Show()
        try:
            fcolor = gdic.VocColor[self.State]
            self.Img.FillColor(fcolor)
            wasTorusJump = (abs(xscr-self.pred_xscr)>=self.env.MAX_SCR_X/2) or (abs(yscr-self.pred_yscr)>=self.env.MAX_SCR_Y/2)

            if(not self.traceOn or self.FirstDraw or wasTorusJump):
                self.Img.PenUp()
                self.FirstDraw = False
            else:
                self.Img.Color(fcolor, fcolor)
                self.Img.PenDown()
            self.Img.SetPos(xscr, yscr)
            self.Img.SetHeading(self.GetAng())
        except:
            gdic.terminate_program("agent.Draw error")
        self.pred_xscr = xscr
        self.pred_yscr = yscr

    def GetX(self): return self.pos[0]
    def GetY(self): return self.pos[1]
    def GetAng(self): return self.pos[2]
