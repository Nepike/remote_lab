#!/usr/bin/env python3
# coding: utf-8

"""
  Map file parser functions

  05.05.2019
  Version 1.1
  LP 10.01.2025

"""
import sys, time, math, os

tQube = -2
tCylinder = -3
tSpot = -4

class TCtl():
    def __init__(self):
        self.title = None
        self.mapname  = None

        self.RobotNum = 0
        self.RobotX = []
        self.RobotY = []
        self.RobotA = []

        self.rat_x = 0.75
        self.rat_y = 0.75
        self.step_x = None
        self.step_y = None

        self.dimX = 0
        self.dimY = 0
        self.num_lines = 0
        self.line_width = 0

        # Ширина и высота окна отображения поля self.MAX_SCR_X self.MAX_SCR_Y
        self.env_scr_X = None
        self.env_scr_Y = None

        # Флаг режима топологии тора self.UseTorusRegime
        # (draw_manege := not env_torus)
        self.env_torus = None

        # Частота процесса self.Rate
        self.env_freq = None

        self.OBJECTS = []

        self.executable_line = ''

ctl = TCtl()

def get_symb_params(c, DICT):
    level = None
    code = 0
    draw_text = False
    res = []

    if c=='Q': return [(None, tQube, None)]
    if c=='R': return [(None, tCylinder, None)]
    if c=='S': return [(None, tSpot, None)]

    for e in DICT:
        if e[0].find(c)>=0:
            level = e[1]
            if e[2]=='' or len(e[2])==0:
                code = None
                if '123456789'.find(c)>=0: code = int(c)
                if ord(c)>=ord('a') and ord(c)<=ord('z'): code = ord(c) - ord('a') + 10
            else:
                code = int(e[2])
            draw_text = e[3]
            res.append((level, code, draw_text))
    return res

def ftr(r): return int(r*1000)/1000.0

def is_eof(f):
    cur = f.tell()    # save current position
    f.seek(0, os.SEEK_END)
    end = f.tell()    # find the size of file
    f.seek(cur, os.SEEK_SET)
    return cur == end

def getline(f):
    res = ''
    while not is_eof(f):
        s = f.readline().strip()
        while s.find('  ') != -1:
            s = s.replace('  ',' ')
        s = s.strip()
        if s.upper()=='<EOF>': return ''
        if s=='!': return ''
        if len(s)==0: continue
        if s[0]==';': continue
        if s[0]=='!':
            s = s[1:]
            if s[-1:] != ',': s = s+','
            ctl.executable_line = ctl.executable_line + s
            ctl.executable_line = ctl.executable_line.strip()
            continue
        res = s
        break
    return res

def ReadMap(mapname):
    ctl.mapname = mapname
    f = open(mapname, 'r', encoding="utf-8")

    # Заголовок
    ctl.title = getline(f)

    # Размер поля self.DIM_X self.DIM_Y
    s = getline(f).split(' ')
    ctl.dimX = int(s[0])
    ctl.dimY = int(s[1])

    # Ширина и высота окна отображения поля self.MAX_SCR_X self.MAX_SCR_Y
    s = getline(f).split(' ')
    ctl.env_scr_X = int(s[0])
    ctl.env_scr_Y = int(s[1])

    # Флаг режима топологии тора self.UseTorusRegime
    s = getline(f).upper()
    ctl.env_torus = (s=='TRUE')

    # Частота процесса self.Rate 100
    s = getline(f)
    ctl.env_freq = int(s)

    # Множители
    s = getline(f).split(' ')
    ctl.rat_x = float(s[0])
    ctl.rat_y = float(s[1])

    ldict = {}
    exec('RDICT=['+ctl.executable_line+']', globals(), ldict)
    RDICT = ldict['RDICT']
    # Все чувствительно к регистру
    # for e in RDICT: e[0] = e[0].upper()
    print(RDICT)

    ctl.num_lines = 0
    while True:
        s = getline(f)
        if len(s)==0: break
        ctl.num_lines += 1

        # Все чувствительно к регистру
        s = s.rstrip()
        if len(s)>ctl.line_width: ctl.line_width = len(s)

        for i in range(len(s)):
            res = get_symb_params(s[i], RDICT)
            for r in res:
                level, code, draw_text = r[0], r[1], r[2]
                if not (level is None) or not (code is None):
                    ctl.OBJECTS.append([i, ctl.num_lines, code, level, draw_text])
            # Robot's position
            if '<>^V'.find(s[i])>=0:
                ctl.RobotNum += 1
                ctl.RobotX.append(i)
                ctl.RobotY.append(ctl.num_lines)
                angle = {'^': 90, 'V': 270, '<': 180, '>': 0}
                ctl.RobotA.append(angle[s[i]])

    f.close()

    ctl.step_x = float(ctl.dimX)/ctl.line_width
    ctl.step_y = float(ctl.dimY)/ctl.num_lines

    print(f"-- total lines = {ctl.num_lines}, width = {ctl.line_width} numrobots = {ctl.RobotNum}")
    print(f"-- draw manege = {not ctl.env_torus}")
    print(f"-- dimX = {ctl.dimX}, dimY = {ctl.dimY}")
    print(f"-- number of objects = {len(ctl.OBJECTS)}")
