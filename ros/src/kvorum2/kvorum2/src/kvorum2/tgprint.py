#!/usr/bin/env python3
# coding: utf-8
'''
https://sites.google.com/site/ev3basic/ev3-basic-programming/going-further/writing-with-the-turtle-part-2
  Modified: Valery Karpov

  LP 12.08.2020
'''

import time, math, turtle

sequence = {
    '0': "1P Qq 20 Qr 20 Qq 0p 10",
    '1': "0A 30 0c 0P cc 0a 4p 0A 5P 2p",
    '2': "0A 3P qs Qq 1A 2p",
    '3': "0P 10 Qr QR 5p 0f fF",
    '4': "1A 0P 2p 2A 0H 0P gG 0A 2p 0a 1A",
    '5': "0P 10 Qr 1a 2a 2p 0a 4A",
    '6': "0N mM 0P Qr 20 Qs 1p 0A 0C dC",
    '7': "1K 0P jJ 0A 2p 0A 0E fF",
    '8': "1P Qr qt Qr 0p 10",
    '9': "0P 10 Qq 20 Qs 1p 0a 2A",

    "A": "0A 0P 2a 2p 6F 0P ef 0f ee 2p 0A",
    "B": "0P 10 Qr 1p 5P QR 5a 4p 0A 20",
    "C": "0N mM 0P Qr 20 Qr 0p 5a",
    "D": "0P 10 Qq 20 Qq 1A 4p 0A 20",
    "E": "0A 2a 0P 1p 0F eE 0A 0P 2A 4A 2p",
    "F": "0A 2a 0P 1F 0p eE 0A 0P 2A 4p 0A 20",
    "G": "0F ef 0P 1a 10 qr 20 qr 0p 3A",
    "H": "0A 2a 0P 2p 0C DC 0P 4p 0a 2a 0P 4p 0A",
    "I": "0P 2p 0A 4A 0P 2p 5A 0P 4p 0A 10",
    "J": "0A 4a 0P 2a 30 qr 0p 0a 0e eE",
    "K": "0A 0P 4p 0a 2C 0P Da dp 0C",
    "L": "0A 4P 8a 2p",
    "M": "0A 0P 4a 0f ee 0e EE 4p 0A",
    "N": "0A 0P 4E Fe 4p 8a",
    "O": "1P Qq 20 Qr 20 Qq 0p 10",
    "P": "0A 0P 4a 10 qr 1p 0A 0C dC",
    "Q": "1P Qq 20 Qr 20 Qq 0p 0A 1a 0c 0P cp 0C",
    "R": "0A 0P 4a 10 qr 1A 0C dp 0C",
    "S": "0a 5P Qs qs 0p 3A",
    "T": "0A 4a 0P 2p 5a 0P 4p 0A 10",
    "U": "0a 8P 30 Qr 3p 7P 5p 0a",
    "V": "0A 4J 0P Jj 0j jp 0J 8a",
    "W": "0A 4P 8e eE 0E Ee 4p 8a",
    "X": "0F 0P fp 0f 6f 0P fp 0F",
    "Y": "1A 0P 2E ep Ee 0e 0P ep 0E 8a",
    "Z": "0A 4a 0P 2F Ff 2p",
    " ": "20", # Обычный пробел
    "-": "0A 2a 0P 2p 0a 2A",
    "#space#": "10" # Пробел между символами в строке
    }

def draw_symbol(t, c, scale = 1, regpolygone = False):
    c = c.upper()
    try:
        sq = sequence[c]
    except:
        sq = sequence[' ']

    t.penup()

    if scale is None: scale = 1
    kkk = scale/20.0

    for n in range(int(len(sq)/3)+1):
        movechar = sq[n*3]
        if movechar.isupper():
            movesign = -1
            movechar = movechar.lower()
        else:
            movesign = 1

        turnchar = sq[n*3+1]
        if turnchar.isupper():
            turnsign = -1
            turnchar = turnchar.lower()
        else:
            turnsign=1

        d = None
        if movechar=="0": pass
        elif movechar=="1": d = 100
        elif movechar=="2": d = 200
        elif movechar=="3": d = 300
        elif movechar=="4": d = 400
        elif movechar=="5": d = -100
        elif movechar=="6": d = -200
        elif movechar=="7": d = -300
        elif movechar=="8": d = -400
        elif movechar=="9": d = 25
        elif movechar=="c": d = movesign*141.4
        elif movechar=="d": d = movesign*282.8
        elif movechar=="e": d = movesign*223.6
        elif movechar=="f": d = movesign*447.2
        elif movechar=="g": d = movesign*316.2
        elif movechar=="j": d = movesign*412.3
        elif movechar=="m": d = movesign*360.6
        elif movechar=="q": # draw arc
            if turnchar=="q": imax=45    # 90° in 45 steps of 2° each
            elif turnchar=="r": imax=90  # 180°
            elif turnchar=="s": imax=135 # 270°
            elif turnchar=="t": imax=180 # 360°
            '''
            for i in range(imax):
                t.forward(kkk*turnsign*3.49)  # 2*pi*100/(4*45) = 3.49
                ca = t.heading()
                a = -movesign*2
                a = a + ca
                t.setheading(a)
            '''
            ang = imax*2
            ang2 = math.radians(ang)
            LA = kkk*turnsign*ang2*100
            rad = -movesign*LA/ang2
            t.circle(rad, ang)

        if not d is None:
            t.forward(d*kkk)

        a = None
        if turnchar=="0": pass
        elif turnchar=="a": a = turnsign*90
        elif turnchar=="b": a = turnsign*180
        elif turnchar=="c": a = turnsign*45
        elif turnchar=="e": a = turnsign*26.565
        elif turnchar=="f": a = turnsign*63.434
        elif turnchar=="g": a = turnsign*18.435
        elif turnchar=="h": a = turnsign*71.565
        elif turnchar=="j": a = turnsign*14.036
        elif turnchar=="k": a = turnsign*75.963
        elif turnchar=="m": a = turnsign*33.69
        elif turnchar=="n": a = turnsign*56.31
        elif turnchar=="p":
            if not regpolygone:
                if turnsign==1: t.penup()
                else: t.pendown()

        if not a is None:
            ca = t.heading()
            a = -a
            a = a + ca
            t.setheading(a)


def draw_string(t, s, scale = None, regpolygone = False):
    s = s.upper()
    for c in s:
        draw_symbol(t, c, scale, regpolygone)
        draw_symbol(t, '#space#', scale, regpolygone)


if __name__ == '__main__':

    don = turtle.Turtle('turtle')
    don.speed(0)
    don.penup()
    don.goto(0, 0)

    draw_string(don, '0123456789-r')
    draw_string(don, 'o0qweq')
    don.setheading(45)
    draw_string(don, 'rty-r')

    print('Done')

    turtle.mainloop()
