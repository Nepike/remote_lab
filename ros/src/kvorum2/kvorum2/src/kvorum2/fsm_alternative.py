#!/usr/bin/env python3
# coding: utf-8

"""@package fsm
    Реализация конечного автомата

    @author Robofob
    @created 17.11.2015
    @modified 05.10.2019
    @version 1.5.2
"""

import random

import logging

logging.getLogger(__name__).addHandler(logging.NullHandler())
logger = logging.getLogger(__name__)

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

""" Правило перехода
"""
class TRule(object):
    def __init__ (self, src_, dest_, cond_, proc_, priority_=100):
        self.src = src_            # Вершина-источник
        self.dest = dest_          # Вершина-приемник
        self.cond = cond_          # Условие перехода
        self.proc = proc_          # Исполняемая процедура
        self.priority = priority_  # Приоритет правила (чем меньше, тем выше приоритет)

    def __str__(self):
        return "{} -> {} [{}] [{}] ({})".format(self.src, self.dest, self.cond, self.proc, self.priority)

    def show(self, title=""):
        logger.info(title + str(self))

""" Автомат
"""
class TAutomaton(object):
    FSM_FINISHED = 0
    FSM_NOT_FINISHED = 1

    def __init__ (self, pname, pstates, pterminals, pq0):
        """ Конструктор

        @param pname      - имя автомата
        @param pstates    - список состояний
        @param pterminals - список терминальных состояний
        @param pq0        - имя начального состояния (оно же раньше было именем автомата)
        """
        self.name = pname    # Имя автомата
        self.rules = []      # Правила перехода
        self.states = []     # Состояния
        self.terminals = []  # Терминальные состояния
        self.starting_state = pq0        # Начальное состояние
        self.current_state = self.starting_state     # Текущее состояние
        self.status = FSM_NOT_FINISHED  # Состояние автомата: FSM_NOT_FINISHED/FSM_FINISHED

        self.trace = False   # Флаг режима диагностического вывода

        # ====== обратная совместимость с библиотекой fsm.py ======
        self.Name = self.name            # Имя автомата
        self.Rules = self.rules          # Правила перехода
        self.States = self.states        # Состояния
        self.Terminals = self.terminals  # Терминальные состояния
        self.q0 = self.starting_state    # Начальное состояние
        self.q = self.current_state      # Текущее состояние
        self.Trace = self.trace          # Флаг режима диагностического вывода
        # =========================================================

        # Копируем
        for e in pstates:
            self.states.append(e)
        for e in pterminals:
            self.terminals.append(e)

        self.steps_counter = 0       # Внутренний счётчик тактов для отсчёта промежутков

    def add_rule(self, r):
        """ Добавить правило перехода к списку
        """
        if type(r) is not TRule:
            raise TypeError('Rules for FSM must be of TRule type')
        self.rules.append(r)

    def find_rule(self, rules):
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
        logger.info("Automaton " + self.name)
        logger.info("States: " + self.states)
        logger.info("Terminals: " + self.terminals)
        logger.info("q0 = " + self.starting_state)
        for e in self.rules:
            e.Log = logger.info
            e.show()
        logger.info("\n")

    # Сброс автомата в исходное состояние
    def reset(self):
        self.current_state = self.starting_state
        self.steps_counter = 0
        self.status = FSM_NOT_FINISHED

    # Шаг (такт) работы автомата
    def step(self):
        self.steps_counter += 1
        if self.trace:
            logger.debug("::STEP NAME = " + self.name + " q = " + self.current_state)

        # Проверка на нахождение в терминальном состоянии
        if self.current_state in self.terminals:
            self.status = FSM_FINISHED
            return FSM_FINISHED

        rules = []          # Список правил, готовых к применению (правила-кандидаты)
        defaultrule = None  # Правило "else"
        # Ищем подходящие правила
        for r in self.rules:
            if r.src == self.current_state:
                if r.cond == "else":
                    defaultrule = r
                else:
                    res = eval(r.cond)
                    if (str(type(res)) != "<type 'bool'>") and (str(type(res)) != "<class 'bool'>"):
                        logger.info(str(type(res)))
                        r.Log = logger.info
                        r.show("rule: ")
                        raise Exception("FSM {} errpr: illegal cond expr type at {}".format(
                            self.name, r.cond))
                    if res:
                        rules.append(r)

        if len(rules)>0:
            # Список правил-кандидатов сформирован
            # Ищем правило с максимальным приоритетом
            crule = self.find_rule(rules)
        else:
            # Выполнение правила "else"
            if defaultrule == None:
                raise Exception("FSM {} error: default rule not found in state {}".format(
                    self.name, self.current_state))
            crule = defaultrule

        if self.trace:
            crule.Log = logger.info
            crule.show("rule: ")

        exec(crule.proc)     # Выполняем процедурную часть
        self.current_state = crule.dest  # Переход в новое состояние

        # ====== обратная совместимость с библиотекой fsm.py ======
        self.q = self.current_state
        # =========================================================

        self.status = FSM_NOT_FINISHED
        return FSM_NOT_FINISHED

    # ====== обратная совместимость с библиотекой fsm.py ======
    addRule = add_rule
    findRule = find_rule
    # =========================================================
