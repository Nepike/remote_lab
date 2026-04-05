#!/usr/bin/env python3
# coding: utf-8

"""@package fsm
    Автомат
    Author: Valery Karpov

    @created 17.11.2015
    @author Robofob
    @date 08.12.2015 / 13.05.2016, 13.03.2022
    @lp 21.01.2018
    @23.03.2024
    @version 1.6
"""

import random, sys, math

#----------------------------------------------------------
# Глобальные переменные и константы
#----------------------------------------------------------

FSM_FINISHED = 0
FSM_NOT_FINISHED = 1

# Направления на цель
G_FWD   =  0 # впереди
G_BACK  =  1 # сзади
G_LEFT  =  2 # слева
G_RIGHT =  3 # справа
G_RAND  =  4 # равновероятное расположение
G_FOUND =  5 # цель достигнута
G_NONE  = -1 # не обнаружено

#----------------------------------------------------------

def error(msg): raise Exception("*** " + msg)

#----------------------------------------------------------

""" Правило перехода
"""
class TRule:
    def __init__ (self, src_, dest_, cond_, proc_, priority_=100):
        self.src = src_          # Вершина-источник
        self.dest = dest_        # Вершина-приемник
        self.cond = cond_        # Условие перехода
        self.proc = proc_        # Исполняемая процедура
        self.priority = priority_  # Приоритет правила (чем меньше, тем выше приоритет)

    def __str__(self) :
        return "{} -> {} [{}] [{}] ({})".format(self.src, self.dest, self.cond, self.proc, self.priority)

    def Log(self, s): print(s)

    def show(self, title=""):
        self.Log(title + str(self))

""" Автомат
"""
class TAutomaton:

    def __init__ (self, pname, pstates, pterminals, pq0):
        """ Конструктор

        @param pname      - имя автомата
        @param pstates    - список состояний
        @param pterminals - список терминальных состояний
        @param pq0        - имя начального состояния (оно же раньше было именем автомата)
        """
        self.Name = pname    # Имя автомата
        self.Rules = []      # Правила перехода
        self.States = []     # Состояния
        self.Terminals = []  # Терминальные состояния
        self.q0 = pq0        # Начальное состояние
        self.q = self.q0     # Текущее состояние
        self.status = FSM_NOT_FINISHED  # Состояние автомата: FSM_NOT_FINISHED/FSM_FINISHED

        self.Trace = False   # Флаг режима диагностического вывода

        # Копируем
        for e in pstates:
            self.States.append(e)
        for e in pterminals:
            self.Terminals.append(e)

        self.T = 0;      # Внутренний счётчик тактов для отсчёта промежутков
        self.TCnt = 0;   # Граница счетчика
        self.GRand = 0   # Значение генератора случайных чисел

    def addRule(self, r):
        """ Добавить правило перехода к списку
        """
        self.Rules.append(r)

    def findRule(self, rules):
        """ Поиск правила с максимальным приоритетом
        Чем меньше priority, тем выше приоритет.
        @param rules Список правил, среди которых ведётся поиск.
        """
        r = rules[0]
        for e in rules:
            if e.priority<r.priority:
                r = e
        return r

    def show(self):
        self.Log("FSM " + self.Name)
        self.Log("States: " + self.States)
        self.Log("Terminals: " + self.Terminals)
        self.Log("q0 = " + self.q0)
        for e in self.Rules: 
            e.Log = self.Log
            e.show()
        self.Log("\n")

    # Сброс автомата в исходное состояние
    def reset(self):
        self.q = self.q0
        self.T = 0
        self.status = FSM_NOT_FINISHED

    def Log(self, s): print(s)

    # Шаг (такт) работы автомата
    def step(self):

        self.GRand = random.randint(0, 100)
        self.T += 1
        if self.Trace:
            self.Log("::STEP NAME = "+self.Name+" q = "+self.q)

        # Проверка на нахождение в терминальном состоянии
        if self.q in self.Terminals:
            self.status = FSM_FINISHED
            return FSM_FINISHED

        rules = []          # Список правил, готовых к применению (правила-кандидаты)
        defaultrule = None  # Правило "else"
        # Ищем подходящие правила
        for r in self.Rules:
            if r.src == self.q:
                if (r.cond == "else"):
                    defaultrule = r
                else:
                    res = eval(r.cond)
                    if type(res) != bool:
                        self.Log(str(type(res)))
                        r.Log = self.Log
                        r.show("rule: ")
                        error("{} ({}): illegal cond expr type at ({})".format(self.Name, self.q, r.cond))
                    if res:
                        rules.append(r)

        if len(rules)>0:
            # Список правил-кандидатов сформирован
            # Ищем правило с максимальным приоритетом
            crule = self.findRule(rules)
        else:
            # Выполнение правила "else"
            if defaultrule == None:
                error("{} ({}): default rule is not found ({})".format(self.Name, self.q, r.cond))
            crule = defaultrule

        if self.Trace:
            crule.Log = self.Log
            crule.show("rule: ")
        try:
            exec(crule.proc)    # Выполняем процедурную часть
        except:
            self.Log("-------------------------------------")
            self.Log("::STEP NAME = "+self.Name+" q = "+self.q)
            self.Log("exec proc '"+crule.proc+"'")
            self.Log("-------------------------------------")
            raise Exception("*** exec fsm error")

        self.q = crule.dest # Переход в новое состояние

        self.status = FSM_NOT_FINISHED
        return FSM_NOT_FINISHED

    # Проверка отсутствия переполнения
    def CheckT(self): return self.T<self.TCnt

    # Установить границу счетчика (таймера)
    def SetTCnt(self, val):
      self.TCnt = val;
      self.T = 0

    # Сброс счетчика (таймера)
    def ResetT(self): self.T = 0

    # Получение случайной величины от 0 до n-1
    def rand(self, n): return self.GRand % n
