#!/usr/bin/env python3
# coding: utf-8

"""
  tshapes.py
  Создание форм для рисования
  Author: Valery Karpov

  Регистрируются формы: "elefant", "hare", "turtle", "square", "yarp"
  06.02.2015, 31.08.2020, 09.11.2021, 22.03.2024
  Version 2.07
  LP 15.08.2024
"""


try:
    from Tkinter import *
except:
    from tkinter import *

from turtle import Shape, Turtle, mainloop, Vec2D as Vec
from kvorum2 import gdic
from kvorum2 import tgprint

GCNV = None

ShapesList = ["turtle", "square"]

################################################################################
#
################################################################################
# draw label
def draw_label(ts, text):
    ts.pu()
    ts.goto(-10, -10)
    ts.setheading(90)
    ts.begin_poly()
    tgprint.draw_string(ts, text, scale = 0.5, regpolygone = True)
    ts.end_poly()
    m = ts.get_poly()
    return m

def CreateShape(name, usrid = None):
    global ShapesList

    if name=="turtle":
        ShapesList.append(name)
        return

    resname = name
    if not usrid is None:
        resname = resname  + '_' + str(usrid)

    print('CreateShape ', resname)
    s = Turtle()
    s.reset()
    s.speed(0)
    s.penup()

    shape = Shape("compound")

    ############################################################################
    # Форма "yarp"
    ############################################################################
    if name=="yarp":
        SZ = 12
        s.begin_poly()
        shp = [(0,0), (SZ,0), (SZ+SZ/2, -SZ/2), (SZ, -SZ), (0, -SZ), (0,0)]
        for p in shp:
            s.goto(Vec(p[1]+SZ/2, p[0]))
        s.end_poly()
        m1 = s.get_poly()
        shape.addcomponent(m1,"", "black")

    ############################################################################
    # Форма "hare"
    ############################################################################
    if name=="hare":
        SZ = 6
        YRAD = SZ/3

        s.ht()
        s.fd(SZ)
        s.lt(90)

        s.begin_poly()
        s.circle(SZ, 180)
        s.end_poly()
        m1 = s.get_poly()

        s.begin_poly()
        s.circle(SZ,180)
        s.end_poly()
        m2 = s.get_poly()

        s.fd(SZ)
        s.begin_poly()
        s.circle(YRAD)
        s.end_poly()
        m3 = s.get_poly()

        s.lt(90)
        s.fd(SZ+YRAD)
        s.rt(90)

        s.begin_poly()
        s.circle(YRAD)
        s.end_poly()
        m4 = s.get_poly()

        shape.addcomponent(m1,"red")
        shape.addcomponent(m2,"grey")
        shape.addcomponent(m3,"green")
        shape.addcomponent(m4,"green")

    ############################################################################
    # Форма "elefant"
    ############################################################################
    if name=="elefant":
        SZ = 6
        YRAD = SZ/3

        s.ht()
        s.fd(SZ)
        s.lt(90)

        s.begin_poly()
        s.circle(SZ, 180)
        s.end_poly()
        m1 = s.get_poly()

        s.begin_poly()
        s.circle(SZ,180)
        s.end_poly()
        m2 = s.get_poly()

        ###
        s.fd(SZ)
        s.begin_poly()
        s.circle(YRAD)
        s.end_poly()
        m3 = s.get_poly()

        ###
        s.lt(90)
        s.fd(SZ+YRAD)
        s.rt(90)

        s.begin_poly()
        s.circle(YRAD)
        s.end_poly()
        m4 = s.get_poly()

        shape.addcomponent(m1,"orange")
        shape.addcomponent(m2,"blue")
        shape.addcomponent(m3,"red")
        shape.addcomponent(m4,"red")

    ############################################################################
    # Форма "p_yellow"
    ############################################################################
    if name=="p_yellow":
        SZ = 6

        s.begin_poly()
        s.circle(SZ, 360)
        s.end_poly()
        m1 = s.get_poly()

        shape.addcomponent(m1, "yellow")

    ############################################################################
    # Форма "p_red"
    ############################################################################
    if name=="p_red":
        SZ = 6

        s.begin_poly()
        s.circle(SZ, 360)
        s.end_poly()
        m1 = s.get_poly()

        shape.addcomponent(m1,"red")

    ############################################################################
    #
    ############################################################################
    if not usrid is None:
        label_shape = draw_label(s, str(usrid))
        shape.addcomponent(label_shape, "", "black")

    s.getscreen().register_shape(resname, shape)
    s.hideturtle()
    ShapesList.append(resname)

#
#
#
class TPen(Turtle):

    def __init__(self, cshape, usrid=None):
        self.shape = cshape
        if (not usrid is None) and cshape!="turtle":
            self.shape = self.shape + '_' + str(usrid)

        if not self.shape in ShapesList:
            CreateShape(cshape, usrid)

        Turtle.__init__(self, shape=self.shape)

        self.Hide()
        self.speed(0)

    def Hide(self): self.hideturtle()

    def Show(self): self.showturtle()

    def SetPos(self, x, y): self.setpos(Vec(x,y))

    def PenDown(self): self.pd()

    def PenUp(self): self.pu()

    def Reset(self): self.reset()

    def Forward(self, n): self.forward(n)

    def Left(self, n): self.left(n)

    def Color(self, c1, c2): self.color(c1, c2)

    def ShapeSize(self, n): self.shapesize(n)

    def FillColor(self, c): self.fillcolor(c)

    def SetHeading(self, a): self.setheading(a)

    # Рисуем прямоугольник
    def DrawRect(self, x0, y0, x1, y1, fillcolor, bordercolor):
        GCNV.create_rectangle(x0, -y0, x1, -y1, outline=bordercolor, fill=fillcolor)

    def Line(self, x0, y0, x1, y1, color = "black"):
        GCNV.create_line(x0, -y0, x1, -y1, fill=color)

    def PutPixel(self, x, y, color):
        self.Line( x, y, x+1, y+1, color)

    def PenSize(self, size): self.pensize(size)

    def Write(self, x, y, text):
        self.pu()
        self.setpos(Vec(x,y))
        self.write(text)
        self.pu()

################################################################################
#
################################################################################

WPen = TPen("turtle")

def CreateTForms(title):
    print("tshapes: Create forms...")
    s = Turtle()
    s.hideturtle()
    w = s.getscreen()

    w.bgcolor("white")
    w.title(title)

    global GCNV
    GCNV = w.getcanvas()

    print("tshapes: Done.")

################################################################################
#
################################################################################
if __name__ == '__main__':

    CreateTForms("Sample")

    '''
    WPen.Line(0,0,100,20, "black")

    WPen.Line(100,20, 100, -20, "black")

    WPen.DrawRect(0, 0, 10, 10, "black", "red")
    WPen.DrawRect(0, 0, -100, -50, "black", "red")

    s0 = TPen("turtle")
    s0.pd()
    s0.setpos(Vec(0,0))
    s0.setpos(Vec(0,200))
    s0.setheading(90)
    s0.showturtle()

    s1 = TPen("hare", 1)
    s1.pd()
    s1.setpos(Vec(0,0))
    s1.setpos(Vec(200,0))
    s1.setheading(0)
    s1.showturtle()

    s2 = TPen("elefant", 2)
    s2.pd()
    s2.setpos(Vec(0,0))
    s2.setpos(Vec(200,200))
    s2.setheading(45)
    s2.showturtle()

    s3 = TPen("square")
    s3.pd()
    s3.setpos(Vec(0,0))
    s3.setpos(Vec(-200,200))
    s3.setheading(45)
    s3.showturtle()

    s4 = TPen("yarp", 12)
    s4.pd()
    s4.setpos(Vec(0,0))
    s4.setpos(Vec(300,0))
    s4.setheading(45)
    s4.showturtle()

    s5 = TPen("turtle")
    s5.pd()
    s5.setpos(Vec(0,0))
    s5.setpos(Vec(0,300))
    s5.setheading(0)
    s5.showturtle()
    '''

    s4 = TPen("hare", 12)
    s4.setpos(Vec(300,0))
    s4.setheading(45)
    s4.showturtle()

    s5 = TPen("yarp", 34)
    s5.setpos(Vec(300,300))
    s5.setheading(45)
    s5.showturtle()
 
    s5.hideturtle()
    s5.pu()
    s5.Write(-100,-100,"123")

    mainloop()
