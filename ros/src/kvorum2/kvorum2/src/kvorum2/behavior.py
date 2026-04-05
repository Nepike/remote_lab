#!/usr/bin/env python3
# coding: utf-8

"""
  FSM Collection
  Author: Valery Karpov

  18.07.2016, 12.09.2020, 23.03.2024
  V 1.1
  LP 15.08.2024

######################################################################
#
# Обобщенные автоматы
#
######################################################################

Для реализации обобщенных автоматов требуется определить следующие методы:

  fsmMove() Метод движения для параметров:
    G_FOUND
    G_FWD   Цель впереди
    G_BACK  Цель сзади
    G_LEFT  Цель слева
    G_RIGHT Цель справа
    G_RAND  Случайное блуждание

  fsmFound() Предикат, определяющий, что объект найден

  fsmDir() Функция, определяющая направление на целевой объект
  Возвращает:
    G_FWD, G_BACK, G_LEFT, G_RIGHT
    G_NONE  Цель не обнаружена

"""

from kvorum2 import gdic
from kvorum2.fsm import *
from kvorum2 import tmurobot as tmu

#----------------------------------------------------------
# Глобальные переменные и константы
#----------------------------------------------------------

CurrRobot = None

################################################################################

#
# Создание некоторых часто используемых автоматов
# Для их использования надо подгружать модуль именно таким образом, чтобы не было проблем с видимостью:
#    exec (open(LIB_PATH+"/fsm.py").read())
# (в этом автомате много неизвестных модулю функций)

def CreateFSM(name):
    if name=="A_REFLEX":
        # Рефлекс на препятствие
        f = TAutomaton("A_REFLEX", ["S", "1", "2", "3","T"], ["T"], "S")
        f.Trace = False
        f.SetTCnt(10)
        f.addRule(TRule("S", "1", "self.fIsLB() and self.fIsRB()", "self.fProcGoBack()"))
        f.addRule(TRule("S", "1", "self.fIsLB()",        "self.fProcGoBack()"))
        f.addRule(TRule("S", "2", "self.fIsRB()",        "self.fProcGoBack()"))
        f.addRule(TRule("S", "T", "else",                "self.fProcStop()"))
        f.addRule(TRule("1", "1", "self.CheckT()", ""))
        f.addRule(TRule("1", "3", "else",                "self.fProcGoRight()"))
        f.addRule(TRule("2", "2", "self.CheckT()", ""))
        f.addRule(TRule("2", "3", "else",                "self.fProcGoLeft()"))
        f.addRule(TRule("3", "3", "self.CheckT()", ""))
        f.addRule(TRule("3", "T", "else",                "self.fProcStop()"))
        return f

    if name=="A_WALK":
        # Свободное блуждание
        f = TAutomaton("A_WALK", ["S", "1", "2", "3"], ["T"], "S")
        f.Trace = False
        f.SetTCnt(5)
        f.addRule(TRule("S", "1", "True",          "self.fProcGoFwd()"))
        f.addRule(TRule("1", "1", "self.CheckT()", ""))
        f.addRule(TRule("1", "2", "True",          "self.fProcRandomTurn()"))
        f.addRule(TRule("2", "2", "self.CheckT()", ""))
        f.addRule(TRule("2", "3", "True",          "self.fProcGoFwd()"))
        f.addRule(TRule("3", "3", "self.CheckT()", ""))
        f.addRule(TRule("3", "S", "True",          ""))
        return f

    if name=="A_SEARCH_FOOD":
        # Поиск пищи
        f = TAutomaton("A_SEARCH_FOOD", ["S", "T", "1", "2", "3"], ["T"], "S")
        f.Trace = False
        f.SetTCnt(10)
        f.addRule(TRule("S", "T", "self.fCheckFood()",  "self.fProcStop()"))
        f.addRule(TRule("S", "1", "True",               "self.fProcGoFwd()"))
        f.addRule(TRule("1", "T", "self.fCheckFood()",  "self.fProcStop()"))
        f.addRule(TRule("1", "1", "self.CheckT()", ""))
        f.addRule(TRule("1", "2", "True",               "self.fProcRandomTurn()"))
        f.addRule(TRule("2", "T", "self.fCheckFood()",  "self.fProcStop()"))
        f.addRule(TRule("2", "2", "self.CheckT()", ""))
        f.addRule(TRule("2", "3", "True",               "self.fProcGoFwd()"))
        f.addRule(TRule("3", "T", "self.fCheckFood()",  "self.fProcStop()"))
        f.addRule(TRule("3", "3", "self.CheckT()", ""))
        f.addRule(TRule("3", "1", "True",               "self.fProcGoFwd()"))
        return f

    if name=="A_SEARCH_SHADOW":
        # Поиск тени
        f = TAutomaton("A_SEARCH_SHADOW", ["S", "1", "2", "3","T"], ["T"], "S")
        f.Trace = False
        f.SetTCnt(10)
        f.addRule(TRule("S", "T", "self.fCheckShadow()", "self.fProcStop()"))
        f.addRule(TRule("S", "1", "True",           "self.fProcGoFwd()"))
        f.addRule(TRule("1", "T", "self.fCheckShadow()", "self.fProcStop()"))
        f.addRule(TRule("1", "1", "self.CheckT()",  ""))
        f.addRule(TRule("1", "2", "True",           "self.fProcRandomTurn()"))
        f.addRule(TRule("2", "T", "self.fCheckShadow()", "self.fProcStop()"))
        f.addRule(TRule("2", "2", "self.CheckT()",  ""))
        f.addRule(TRule("2", "3", "True",           "self.fProcGoFwd()"))
        f.addRule(TRule("3", "T", "self.fCheckShadow()", "self.fProcStop()"))
        f.addRule(TRule("3", "3", "self.CheckT()",  ""))
        f.addRule(TRule("3", "1", "True",           "self.fProcGoFwd()"))
        return f

    if name=="A_EAT":
        # Поедание пищи
        f = TAutomaton("A_EAT", ["S", "1", "2", "3", "4", "T"], ["T"], "S")
        f.Trace = False
        f.SetTCnt(5)
        f.addRule(TRule("S", "1", "True",          "self.fProcStop()"))
        f.addRule(TRule("1", "1", "self.CheckT()", ""))
        f.addRule(TRule("1", "2", "True",          "self.fProcEat()"))
        f.addRule(TRule("2", "2", "self.CheckT()", ""))
        f.addRule(TRule("2", "T", "True",          "self.fProcStop()"))
        return f

    if name=="A_SLEEP":
        # Сон
        f = TAutomaton("A_SLEEP", ["S", "1", "2", "3", "T"], ["T"], "S")
        f.Trace = False
        f.SetTCnt(5)
        f.addRule(TRule("S", "1", "True",          "self.fProcStop()"))
        f.addRule(TRule("1", "1", "self.CheckT()", ""))
        f.addRule(TRule("1", "2", "True",          "self.fProcRandomTurn()"))
        f.addRule(TRule("2", "2", "self.CheckT()", ""))
        f.addRule(TRule("2", "3", "True",          "self.fProcStop()"))
        f.addRule(TRule("3", "3", "self.CheckT()", ""))
        f.addRule(TRule("3", "1", "True",          ""))
        return f

    if name=="A_MOVE_TO_OBST":
        # Движение к препятствию
        f = TAutomaton("A_MOVE_TO_OBST", ["S", "1", "2", "3", "T"], ["T"], "S")
        f.Trace = False
        f.SetTCnt(10)
        f.addRule(TRule("S", "T", "self.fCheckObstFwd()",   "self.fProcStop()"))
        f.addRule(TRule("S", "S", "self.fCheckObstLeft()",  "self.fProcGoLeft()"))
        f.addRule(TRule("S", "S", "self.fCheckObstRight()", "self.fProcGoRight()"))
        f.addRule(TRule("S", "1", "True",                   "self.fProcGoFwd()"))

        f.addRule(TRule("1", "T", "self.fCheckObstFwd()",   "self.fProcStop()"))
        f.addRule(TRule("1", "S", "self.fCheckObst()",      ""))
        f.addRule(TRule("1", "1", "self.CheckT()",     ""))
        f.addRule(TRule("1", "2", "True",                   "self.fProcRandomTurn()"))

        f.addRule(TRule("2", "T", "self.fCheckObstFwd()",   "self.fProcStop()"))
        f.addRule(TRule("2", "S", "self.fCheckObst()",      ""))
        f.addRule(TRule("2", "2", "self.CheckT()",     ""))
        f.addRule(TRule("2", "3", "True",                    "self.fProcGoFwd()"))

        f.addRule(TRule("3", "T", "self.fCheckObstFwd()",   "self.fProcStop()"))
        f.addRule(TRule("3", "S", "self.fCheckObst()",      ""))
        f.addRule(TRule("3", "3", "self.CheckT()",     ""))
        f.addRule(TRule("3", "1", "True",              "self.fProcGoFwd()"))
        return f

    if name=="A_ESCAPE":
        # Убегание от опасности (от препятствия)
        f = TAutomaton("A_ESCAPE", ["S", "1", "2", "3", "4", "T"], ["T"], "S")
        f.Trace = False
        f.SetTCnt(5)

        f.addRule(TRule("S", "T", "self.fCheckNoObst()",           "self.fProcStop()"))
        f.addRule(TRule("S", "1", "self.fCheckFreeFwd()",          "self.fProcGoBack()"))
        f.addRule(TRule("S", "3", "self.fCheckFreeLeft()",         "self.fProcGoLeft()"))
        f.addRule(TRule("S", "4", "self.fCheckFreeRight()",        "self.fProcGoRight()"))
        f.addRule(TRule("S", "S", "True",                          "self.fProcGoBack()"))

        f.addRule(TRule("1", "1", "self.fCheckTcntAndFreeFwd()",   ""))
        f.addRule(TRule("1", "2", "True",                     "self.fProcGoLeft()"))

        f.addRule(TRule("2", "2", "self.CheckT()",            ""))
        f.addRule(TRule("2", "S", "True",                     "self.fProcStop()"))

        f.addRule(TRule("3", "3", "self.fCheckTcntAndFreeLeft()",  ""))
        f.addRule(TRule("3", "S", "True",                     "self.fProcStop()"))

        f.addRule(TRule("4", "4", "self.fCheckTcntAndFreeRight()", ""))
        f.addRule(TRule("4", "S", "True",                     "self.fProcStop()"))
        return f

    gdic.error("fsm/behavior.CreateFSM: "+name+" not found")
    return None

#
# Обобщенный поисковый автомат
#
def CreateGBPSearch(name): return CreateGBPSearchFwdRot(name)


def CreateGBPSearchFwd(name):

    f = TAutomaton(name, ["S", "W", "R0", "RF", "R1", "R2", "R3", "R4", "R6", "T"], ["T"], "S")
    f.RTM = 40
    f.SetTCnt(f.RTM)

    f.fProcGoLeft = fProcGoLeft
    f.fProcGoRight = fProcGoRight
    f.fProcGoFwd = fProcGoFwd

    f.addRule(TRule("S", "W",  "True", "self.ResetT()"))

    f.addRule(TRule("W", "T",  "self.fsmFound()",        "self.fsmMove(G_FOUND)"))
    f.addRule(TRule("W", "W",  "self.fsmDir()==G_FWD",   "self.fsmMove(G_FWD)"))     # Цель впереди
    f.addRule(TRule("W", "W",  "self.fsmDir()==G_BACK",  "self.fsmMove(G_BACK)"))    # Цель сзади
    f.addRule(TRule("W", "W",  "self.fsmDir()==G_LEFT",  "self.fsmMove(G_LEFT)"))    # Цель слева
    f.addRule(TRule("W", "W",  "self.fsmDir()==G_RIGHT", "self.fsmMove(G_RIGHT)"))   # Цель справа
    f.addRule(TRule("W", "R0", "else",                   "self.fsmMove(G_FWD)"))     # Цель не обнаружена

    f.addRule(TRule("R0", "RF", "True", "self.SetTCnt(self.RTM); self.ResetT(); self.fProcGoFwd()"))
    f.addRule(TRule("RF", "T",  "self.fsmFound()",  "self.fsmMove(G_FOUND)"))
    f.addRule(TRule("RF", "W",  "self.fsmDir()>=0", ""))
    f.addRule(TRule("RF", "R1", "not self.CheckT()", ""))
    f.addRule(TRule("RF", "RF", "else", ""))

    f.addRule(TRule("R1", "R2","True", "self.SetTCnt(self.RTM); self.ResetT(); self.fProcGoLeft()"))

    f.addRule(TRule("R2", "T",  "self.fsmFound()",  "self.fsmMove(G_FOUND)"))
    f.addRule(TRule("R2", "W",  "self.fsmDir()>=0", ""))
    f.addRule(TRule("R2", "R4", "not self.CheckT()", "self.SetTCnt(self.RTM*2); self.ResetT(); self.fProcGoRight()"))
    f.addRule(TRule("R2", "R2", "else", ""))

    f.addRule(TRule("R4", "T",  "self.fsmFound()",  "self.fsmMove(G_FOUND)"))
    f.addRule(TRule("R4", "W",  "self.fsmDir()>=0", ""))
    f.addRule(TRule("R4", "R6", "not self.CheckT()", "self.SetTCnt(self.RTM); self.ResetT(); self.fProcGoLeft()"))
    f.addRule(TRule("R4", "R4", "else", ""))

    f.addRule(TRule("R6", "T",  "self.fsmFound()",  "self.fsmMove(G_FOUND)"))
    f.addRule(TRule("R6", "W",  "self.fsmDir()>=0", ""))
    f.addRule(TRule("R6", "W",  "not self.CheckT()", ""))
    f.addRule(TRule("R6", "R6", "else", ""))

    return f

def CreateGBPSearchRotFwd(name):

    f = TAutomaton(name, ["S", "W", "L1", "L1_0", "L2_9", "F_0", "F", "R_0", "R1", "R2", "R3", "T"], ["T"], "S")
    f.RTM = 40
    f.SetTCnt(f.RTM)

    f.fProcGoLeft = fProcGoLeft
    f.fProcGoRight = fProcGoRight
    f.fProcGoFwd = fProcGoFwd

    f.addRule(TRule("S", "W",  "True", "self.ResetT()"))

    f.addRule(TRule("W", "T",  "self.fsmFound()",        "self.fsmMove(G_FOUND)"))
    f.addRule(TRule("W", "W",  "self.fsmDir()==G_FWD",   "self.fsmMove(G_FWD)"))     # Цель впереди
    f.addRule(TRule("W", "W",  "self.fsmDir()==G_BACK",  "self.fsmMove(G_BACK)"))    # Цель сзади
    f.addRule(TRule("W", "W",  "self.fsmDir()==G_LEFT",  "self.fsmMove(G_LEFT)"))    # Цель слева
    f.addRule(TRule("W", "W",  "self.fsmDir()==G_RIGHT", "self.fsmMove(G_RIGHT)"))   # Цель справа
    f.addRule(TRule("W", "L1_0", "else",                 "self.fsmMove(G_FWD)"))     # Цель не обнаружена

    f.addRule(TRule("L1_0", "L1","True", "self.SetTCnt(self.RTM); self.ResetT(); self.fProcGoLeft()"))
    f.addRule(TRule("L1", "T",  "self.fsmFound()",  "self.fsmMove(G_FOUND)"))
    f.addRule(TRule("L1", "W",  "self.fsmDir()>=0", ""))
    f.addRule(TRule("L1", "R_0", "not self.CheckT()", ""))
    f.addRule(TRule("L1", "L1", "else", ""))

    f.addRule(TRule("R_0", "R","True", "self.SetTCnt(self.RTM*2); self.ResetT(); self.fProcGoRight()"))
    f.addRule(TRule("R", "T",  "self.fsmFound()",  "self.fsmMove(G_FOUND)"))
    f.addRule(TRule("R", "W",  "self.fsmDir()>=0", ""))
    f.addRule(TRule("R", "L2_0", "not self.CheckT()", ""))
    f.addRule(TRule("R", "R", "else", ""))

    f.addRule(TRule("L2_0", "L2","True", "self.SetTCnt(self.RTM); self.ResetT(); self.fProcGoLeft()"))    
    f.addRule(TRule("L2", "T",  "self.fsmFound()",  "self.fsmMove(G_FOUND)"))
    f.addRule(TRule("L2", "W",  "self.fsmDir()>=0", ""))
    f.addRule(TRule("L2", "F_0",  "not self.CheckT()", ""))
    f.addRule(TRule("L2", "L2", "else", ""))

    f.addRule(TRule("F_0", "F", "True", "self.SetTCnt(self.RTM); self.ResetT(); self.fProcGoFwd()"))    
    f.addRule(TRule("F", "T",  "self.fsmFound()",  "self.fsmMove(G_FOUND)"))
    f.addRule(TRule("F", "W",  "self.fsmDir()>=0", ""))
    f.addRule(TRule("F", "W", "not self.CheckT()", ""))
    f.addRule(TRule("F", "F", "else", ""))

    return f

def CreateGBPSearchFwdRot(name):

    f = TAutomaton(name, ["S", "W", "F_0", "F", "L1_0", "L1", "R_0", "R", "L2_0", "L2", "T"], ["T"], "S")
    f.RTM = 40
    f.SetTCnt(f.RTM)

    f.fProcGoLeft = fProcGoLeft
    f.fProcGoRight = fProcGoRight
    f.fProcGoFwd = fProcGoFwd

    f.addRule(TRule("S", "W",  "True", "self.ResetT()"))

    f.addRule(TRule("W", "T",  "self.fsmFound()",        "self.fsmMove(G_FOUND)"))
    f.addRule(TRule("W", "W",  "self.fsmDir()==G_FWD",   "self.fsmMove(G_FWD)"))   # Цель впереди
    f.addRule(TRule("W", "W",  "self.fsmDir()==G_BACK",  "self.fsmMove(G_BACK)"))  # Цель сзади
    f.addRule(TRule("W", "W",  "self.fsmDir()==G_LEFT",  "self.fsmMove(G_LEFT)"))  # Цель слева
    f.addRule(TRule("W", "W",  "self.fsmDir()==G_RIGHT", "self.fsmMove(G_RIGHT)")) # Цель справа
    f.addRule(TRule("W", "F_0", "else",                  "self.fsmMove(G_FWD)"))   # Цель не обнаружена

    f.addRule(TRule("F_0", "F", "True", "self.SetTCnt(self.RTM); self.ResetT(); self.fProcGoFwd()"))
    f.addRule(TRule("F", "T",  "self.fsmFound()",  "self.fsmMove(G_FOUND)"))
    f.addRule(TRule("F", "W",  "self.fsmDir()>=0", ""))
    f.addRule(TRule("F", "L1_0", "not self.CheckT()", ""))
    f.addRule(TRule("F", "F", "else", ""))

    f.addRule(TRule("L1_0", "L1","True", "self.SetTCnt(self.RTM); self.ResetT(); self.fProcGoLeft()"))
    f.addRule(TRule("L1", "T",  "self.fsmFound()",  "self.fsmMove(G_FOUND)"))
    f.addRule(TRule("L1", "W",  "self.fsmDir()>=0", ""))
    f.addRule(TRule("L1", "R_0", "not self.CheckT()", ""))
    f.addRule(TRule("L1", "L1", "else", ""))

    f.addRule(TRule("R_0", "R","True", "self.SetTCnt(self.RTM*2); self.ResetT(); self.fProcGoRight()"))
    f.addRule(TRule("R", "T",  "self.fsmFound()",  "self.fsmMove(G_FOUND)"))
    f.addRule(TRule("R", "W",  "self.fsmDir()>=0", ""))
    f.addRule(TRule("R", "L2_0", "not self.CheckT()", ""))
    f.addRule(TRule("R", "R", "else", ""))

    f.addRule(TRule("L2_0", "L2","True", "self.SetTCnt(self.RTM); self.ResetT(); self.fProcGoLeft()"))
    f.addRule(TRule("L2", "T",  "self.fsmFound()",  "self.fsmMove(G_FOUND)"))
    f.addRule(TRule("L2", "W",  "self.fsmDir()>=0", ""))
    f.addRule(TRule("L2", "W",  "not self.CheckT()", ""))
    f.addRule(TRule("L2", "L2", "else", ""))

    return f

def CreateGBPSearchRand(name):

    f = TAutomaton(name, ["S", "W", "R1", "R2", "R3", "T"], ["T"], "S")
    f.SetTCnt(10)
    f.addRule(TRule("S", "W",  "True", "self.ResetT()"))

    f.addRule(TRule("W", "T",  "self.fsmFound()",        "self.fsmMove(G_FOUND)"))
    f.addRule(TRule("W", "W",  "self.fsmDir()==G_FWD",   "self.fsmMove(G_FWD)"))                # Цель впереди
    f.addRule(TRule("W", "W",  "self.fsmDir()==G_BACK",  "self.fsmMove(G_BACK)"))               # Цель сзади
    f.addRule(TRule("W", "W",  "self.fsmDir()==G_LEFT",  "self.fsmMove(G_LEFT)"))               # Цель слева
    f.addRule(TRule("W", "W",  "self.fsmDir()==G_RIGHT", "self.fsmMove(G_RIGHT)"))              # Цель справа
    f.addRule(TRule("W", "R1", "else",                   "self.fsmMove(G_FWD); self.ResetT()")) # Цель не обнаружена

    f.addRule(TRule("R1", "T", "self.fsmFound()",        "self.fsmMove(G_FOUND)"))
    f.addRule(TRule("R1", "W", "self.fsmDir()>=0",      ""))
    f.addRule(TRule("R1", "R1","self.CheckT()", ""))
    f.addRule(TRule("R1", "R2","True",                   "self.fsmMove(G_RAND); self.ResetT()"))

    f.addRule(TRule("R2", "T", "self.fsmFound()",       "self.fsmMove(G_FOUND)"))
    f.addRule(TRule("R2", "W", "self.fsmDir()>=0",      ""))
    f.addRule(TRule("R2", "R2","self.CheckT()", ""))
    f.addRule(TRule("R2", "R3","True",                  "self.fsmMove(G_FWD); self.ResetT()"))

    f.addRule(TRule("R3", "T", "self.fsmFound()",       "self.fsmMove(G_FOUND)"))
    f.addRule(TRule("R3", "W", "self.fsmDir()>=0",      ""))
    f.addRule(TRule("R3", "R3","self.CheckT()", ""))
    f.addRule(TRule("R3", "W", "True",          ""))

    return f

#
# Обобщенный "убегающий" автомат
#
def CreateGBPEscape(name):

    f = TAutomaton(name, ["S", "L", "R", "3","T"], ["T"], "S")
    f.SetTCnt(15)
    f.addRule(TRule("S", "L", "self.fsmDir()==G_FWD",   "self.fsmMove(G_FWD);"))
    f.addRule(TRule("S", "L", "self.fsmDir()==G_LEFT",  "self.fsmMove(G_FWD); self.ResetT()"))
    f.addRule(TRule("S", "R", "self.fsmDir()==G_RIGHT", "self.fsmMove(G_FWD); self.ResetT()"))
    f.addRule(TRule("S", "T", "else",                   "self.fsmMove(G_NONE); self.ResetT()"))
    f.addRule(TRule("L", "L", "self.CheckT()", ""))
    f.addRule(TRule("L", "3", "else",                   "self.fsmMove(G_LEFT); self.ResetT()"))
    f.addRule(TRule("R", "R", "self.CheckT()", ""))
    f.addRule(TRule("R", "3", "else",                   "self.fsmMove(G_RIGHT); self.ResetT()"))
    f.addRule(TRule("3", "3", "self.CheckT()", ""))
    f.addRule(TRule("3", "S", "else",                   "self.fsmMove(G_NONE); self.ResetT()"))
    return f

#
# Обобщенный "обходящий" автомат (реакция на препятствие, например)
#
def CreateGBPBypass(name):

    f = TAutomaton(name, ["S", "T"], ["T"], "S")
    f.SetTCnt(50)
    f.TurnLeft = fProcGoLeft
    f.addRule(TRule("S", "S", "not self.CheckT()",      "self.TurnLeft(15-random.randint(0, 30)); self.ResetT()"))
    f.addRule(TRule("S", "S", "self.fsmDir()==G_FWD",   "self.TurnLeft(15-random.randint(0, 30))"))
    f.addRule(TRule("S", "S", "self.fsmDir()==G_LEFT",  "self.fsmMove(G_LEFT)"))
    f.addRule(TRule("S", "S", "self.fsmDir()==G_RIGHT", "self.fsmMove(G_RIGHT)"))
    f.addRule(TRule("S", "T", "else",                   "self.fsmMove(G_NONE)"))

    return f

######################################################################
#
# Обобщенные процедуры
#
######################################################################

#
# Движение
#
def fProcGoFwd(): CurrRobot.Make(tmu.PROC_GOFWD, 0)

def fProcRandomTurn(ang=0):
    if(CurrRobot.curr_fsm.rand(2)==0): CurrRobot.Make(tmu.PROC_GOLEFT, ang)
    else: CurrRobot.Make(tmu.PROC_GORIGHT, ang)

def fProcGoBack(): CurrRobot.Make(tmu.PROC_GOBACK, 0)

def fProcGoStop(): CurrRobot.Make(tmu.PROC_STOP, 0)

def fProcGoRight(ang=0): CurrRobot.Make(tmu.PROC_GORIGHT, ang)

def fProcGoLeft(ang=0): CurrRobot.Make(tmu.PROC_GOLEFT, ang)

def fProcEat(level): CurrRobot.Make(tmu.PROC_EAT, level)

#
# Обобщенные действия
#
def IsRandomEvent():
    # Вводим случайную компоненту (каждое TC-действие - кривое)
    TC = 5
    try:
        IsRandomEvent.cnt += 1
    except:
        IsRandomEvent.cnt = 0
    if (IsRandomEvent.cnt % TC == TC/2):
        return True
    return False

# Приближение
def fMoveToGeneralProc(dr):
    # Вводим случайную компоненту
    # sic
    if(IsRandomEvent() and dr!=G_FOUND):
        fProcRandomTurn()
        return
    if(dr==G_FOUND):   fProcGoStop()
    elif(dr==G_FWD):   fProcGoFwd()
    elif(dr==G_BACK):  fProcGoLeft()
    elif(dr==G_LEFT):  fProcGoLeft()
    elif(dr==G_RIGHT): fProcGoRight()
    elif(dr==G_NONE):  fProcGoFwd()
    elif(dr==G_RAND):  fProcRandomTurn()

# Убегание
def fEscapeGeneralProc(dr):
    # Вводим случайную компоненту
    if(IsRandomEvent()):
        fProcRandomTurn()
        return

    if(dr==G_FWD):     fProcGoBack()
    elif(dr==G_LEFT):  fProcGoRight()
    elif(dr==G_RIGHT): fProcGoLeft()
    elif(dr==G_NONE):  fProcGoStop()
    elif(dr==G_RAND):  fProcRandomTurn()
