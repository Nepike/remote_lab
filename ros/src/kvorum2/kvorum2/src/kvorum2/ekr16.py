#!/usr/bin/env python3
# coding: utf-8

"""
  ekernel ekr16
  Эмоциональное ядро
  Author: Valery Karpov

  06.02.2015, 08.09.2016, 27.04.2018, 23.03.2024
  Version 3.04
  LP 15.08.2024

"""

import sys
from kvorum2.ctb import *

# Процедуры
( PROC_SEARCH_FOOD,       # Поиск пищи
  PROC_EAT,               # Поедание пищи
  PROC_ESCAPE,            # Убегание от препятствия
  PROC_SEARCH_SHADOW,     # Поиск тени
  PROC_SLEEP,             # Сон
  PROC_WALK,              # Свободное блуждание
  PROC_MOVE_TO_OBSTACLE,  # Движение к препятствию
  PROC_NONE
) = range(8)

ProcNames = {
              PROC_SEARCH_FOOD:      "S.FOOD",
              PROC_EAT:              "EAT",
              PROC_ESCAPE:           "ESCAPE",
              PROC_SEARCH_SHADOW:    "S.SHADOW",
              PROC_SLEEP:            "SLEEP",
              PROC_WALK:             "WALK",
              PROC_MOVE_TO_OBSTACLE: "MV_OBST",
              PROC_NONE:             "None" }
              
################################################################################
# Параметры эмоциональной модели
################################################################################
class TTEParams:
    def __init__(self, kfb = 0.2, kem = 0.2, kext = 1, kinhib = 10):
        self.coeff_fb = kfb
        self.coeff_em = kem
        self.excitation = kext
        self.inhibit = kinhib
        self.timelim = None
        self.logfile = None

    def __str__(self):
        s = "coeff_fb: " + str(self.coeff_fb) + " coeff_em: " + str(self.coeff_em) + " excitation: " + str(self.excitation)
        s = s + " inhibit: " + str(self.inhibit) + " logfile: " + self.logfile + " timelim: " + str(self.timelim) + "\n"
        return s

    def show(self):
        print("::: ET Parameters:")
        print("::: coeff_fb:", self.coeff_fb, "coeff_em:", self.coeff_em, "excitation:", self.excitation, "inhibit:", self.inhibit, "logfile:", self.logfile, "timelim:",  self.timelim)

################################################################################
# Потребности
################################################################################
class TNeeds:
    def __init__(self):
        self.food     = 0.4 #
        self.comfort  = 0.3
        self.save     = 0.5

################################################################################
# Сенсоры
################################################################################
class TSens:
    def __init__(self):
        self.obstacle = 0
        self.danger  = 0
        self.food    = 0
        self.hungry  = 0

################################################################################
# TEmoKernel
################################################################################

class TEmoKernel:

    def __init__(self, params):
        self.coeff_fb = params.coeff_fb     # 0.2
        self.coeff_em = params.coeff_em     # 0.2
        self.excitation = params.excitation # 1
        self.inhibit = params.inhibit       # 10

        self.rb_gate_food = TRingBuffer(self.inhibit)
        self.rb_gate_obstacle = TRingBuffer(self.inhibit)
        self.rb_gate_danger = TRingBuffer(self.inhibit)

        self.rb_proc_eat = TRingBuffer(self.inhibit)
        self.rb_proc_move_to_obstacle = TRingBuffer(self.inhibit)
        self.rb_proc_walk = TRingBuffer(self.inhibit)
        self.rb_proc_escape = TRingBuffer(self.inhibit)
        self.rb_proc_search_shadow = TRingBuffer(self.inhibit)
        self.rb_proc_sleep = TRingBuffer(self.inhibit)
        self.rb_proc_search_food = TRingBuffer(self.inhibit)

        self.PROC_NUM = len(ProcNames)
        self.EM = []
        self.PROC = []
        self.etOUT = []

        for i in range(0, self.PROC_NUM):
            self.EM.append(0)
            self.PROC.append(0)
            self.etOUT.append(0)

    def Step(self, sens, needs, params):

        if((sens is None) or (needs is None)): return (None, None, None)

        need_food = TCt(needs.food, 1)
        need_save = TCt(needs.save, 1)
        need_comfort = TCt(needs.comfort, 1)

        sen_food = TCt(sens.food, 1)
        sen_obstacle = TCt(sens.obstacle, 1)
        sen_danger = TCt(sens.danger, 1)
        sen_hungry = TCt(sens.hungry, 1)

        self.gate_food     = self.rb_gate_food     << sen_food + self.EM[PROC_EAT]
        self.gate_obstacle = self.rb_gate_obstacle << sen_obstacle + self.EM[PROC_MOVE_TO_OBSTACLE] + self.EM[PROC_WALK]
        self.gate_danger   = self.rb_gate_danger   << sen_danger + self.EM[PROC_ESCAPE]

        ########################################################################
        # Поедание пищи: proc_eat_food :- need_food & self.gate_food & sen_hungry
        self.PROC[PROC_EAT] = self.rb_proc_eat << self.excitation*(need_food & self.gate_food & sen_hungry)

        # Движение к препятствию: proc_move_to_obstacle :- self.gate_obstacle & need_food & !self.gate_food
        self.PROC[PROC_MOVE_TO_OBSTACLE] = \
            self.rb_proc_move_to_obstacle << self.excitation*(need_food & ~self.gate_obstacle & ~self.gate_food & sen_hungry)

        # Свободное блуждание: proc_walk :- need_comfort & !self.gate_obstacle
        self.PROC[PROC_WALK] = self.rb_proc_walk << self.excitation*(need_comfort & ~self.gate_obstacle)

        # Убегание от препятствия: proc_run_away :- need_save & self.gate_obstacle
        self.PROC[PROC_ESCAPE] = self.rb_proc_escape << self.excitation*(need_save & self.gate_obstacle)

        # Поиск тени: PROC_SEARCH_SHADOW :- need_comfort & self.gate_danger
        self.PROC[PROC_SEARCH_SHADOW] = self.rb_proc_search_shadow << self.excitation*(need_comfort & self.gate_danger)

        # Сон: PROC_SLEEP :- need_comfort & !self.gate_danger
        self.PROC[PROC_SLEEP] = self.rb_proc_sleep << self.excitation*(need_comfort & ~self.gate_danger)

        # Поиск пищи: PROC_SEARCH_FOOD :- need_food & !self.gate_food & sen_hungry
        self.PROC[PROC_SEARCH_FOOD] = self.rb_proc_search_food << self.excitation*(need_food & ~self.gate_food & sen_hungry)

        ########################################################################
        # Выходные значения
        ########################################################################
        V = []
        self.FACT = []
        for i in range(0, self.PROC_NUM):
            V.append(0)
            self.FACT.append(0)

        for i in range(0, self.PROC_NUM):
            if(isinstance(self.PROC[i], TCt)):
                S = 0
                for j in range(0, self.PROC_NUM):
                    if(i!=j): S += self.etOUT[j]
                res = self.PROC[i] - self.coeff_fb * S
                V[i] = res.a

        for i in range(0, self.PROC_NUM):
            self.etOUT[i] = V[i]

        ########################################################################
        # self.FACT Out
        # Выводить вектор etFACT не надо - он неинтересный
        ########################################################################
        nmax = 0
        for i in range(0, self.PROC_NUM):
            if(self.etOUT[i]>self.etOUT[nmax]): nmax = i
            self.FACT[i] = 0

        self.FACT[nmax] = 1

        currProcedure = nmax
        ########################################################################
        # Вычисляем эмоции
        ########################################################################
        EMOTION = 0
        for i in range(0, self.PROC_NUM):
            if(isinstance(self.PROC[i], TCt)):
                self.EM[i] = self.coeff_em*(self.FACT[i] - self.PROC[i].a)
                EMOTION += self.EM[i]

        ########################################################################
        #
        ########################################################################

        # Определяем номер процедуры
        cpnmax = 0
        for i in range(0, self.PROC_NUM):
            if(isinstance(self.PROC[i], TCt)):
                if(self.PROC[i]>self.PROC[nmax]): cpnmax = i
        return (currProcedure, EMOTION, cpnmax)
 

    def ShowStatus(self, sens, needs):
        print("needs:   food={:4.2f}, save={:4.2f}, comfort={:4.2f}".format( \
            needs.food, needs.save, needs.comfort))        
        print("sensors: food={:4.2f}, hungry={:4.2f}, obst={:4.2f}, danger={:4.2f}".format( \
            sens.food, sens.hungry, sens.obstacle, sens.danger))        
        print("gates:   food={:4},              obst={:4}, danger={:4}".format( \
            self.gate_food, self.gate_obstacle, self.gate_danger))

        print('      ', end='')
        for i in range(0, self.PROC_NUM):
            print('{:>8} '.format(ProcNames[i]), end='')
        print('')            
        print("".rjust(self.PROC_NUM*9+6,'-'))
        printvect("FACT: ", self.FACT, fmt="{:8.2f}")
        printvect("etOUT:", self.etOUT, fmt="{:8.2f}")
        printvect("EM:   ", self.EM, fmt="{:8.2f}")

        print("PROC: ", end='')
        for i in range(0, self.PROC_NUM):
            r = Ct2Float(self.PROC[i])
            print('{:8.2f} '.format(r), end='')
        print('')            

def Ct2Float(c):
    if (type(c)==int or type(c)==float): r = float(c)
    else: r = float(c.a)
    return r

def printvect(text, V, size=None, fmt=None):
    s = ""
    if(size is None):
        for e in V:
            if not fmt is None: sn = fmt.format(e)
            else: sn = str(e)
            s = s+sn+" "
    else:
        for i in range(0, size):
            if not fmt is None: sn = fmt.format(V[i])
            else: sn = str(V[i])
            s = s+sn+" "
    print(text+s)
