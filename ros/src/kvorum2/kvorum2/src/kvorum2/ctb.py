#!/usr/bin/env python3
# coding: utf-8

"""
  ctb.py
  Author: Valery Karpov

  06.02.15
  Version 3.04
  LP 27.04.2018

"""

################################################################################
# TCt
################################################################################

class TCt:
    #  Конструктор
    def __init__(self, v, div = 1):
        self.a = float(v)/float(div)

    # "+"
    def __add__(self, other):
        if isinstance(other, TCt): r = self.a + other.a
        else: r = self.a + other
        if r<0: r = 0
        if r>1: r = 1
        return TCt(r)

    def __radd__(self, other):
        if isinstance(other, TCt): r = self.a + other.a
        else: r = self.a + other
        if r<0: r = 0
        if r>1: r = 1
        return TCt(r)

    # "-"
    def __sub__(self, other):
        if isinstance(other, TCt): r = self.a - other.a
        else: r = self.a - other
        if r<0: r = 0
        if r>1: r = 1
        return TCt(r)

    def __rsub__(self, other):
        if isinstance(other, TCt): r = self.a - other.a
        else: r = self.a - other
        if r<0: r = 0
        if r>1: r = 1
        return TCt(r)

    # "=="
    def __eq__(self, other):
        if isinstance(other, TCt): return  self.a == other.a
        return self.a == other

    # ">"
    def __gt__(self, other):
        if isinstance(other, TCt): return self.a > other.a
        return self.a > other

    # ">="
    def __ge__(self, other):
        if isinstance(other, TCt): return self.a >= other.a
        return self.a >= other

    # "<"
    def __lt__(self, other):
        if isinstance(other, TCt): return self.a < other.a
        return self.a < other

    # "<="
    def __le__(self, other):
        if isinstance(other, TCt): return self.a <= other.a
        return self.a <= other

    # "*"
    def __mul__(self, other):
        if isinstance(other, TCt): r = self.a * other.a
        else: r = self.a * other
        if r<0: r = 0
        if r>1: r = 1
        return TCt(r)

    def __rmul__(self, other):
        if isinstance(other, TCt): r = self.a * other.a
        else: r = self.a * other
        if r<0: r = 0
        if r>1: r = 1
        return TCt(r)

    # "И" &
    def __and__(self, other):
        if isinstance(other, TCt): return TCt(min(self.a, other.a))
        else: return TCt(min(self.a, other))
    # "И" &
    def __rand__(self, other):
        if isinstance(other, TCt): return TCt(min(self.a, other.a))
        else: return TCt(min(self.a, other))

    # "ИЛИ" |
    def __or__(self, other):
        if isinstance(other, TCt): return TCt(max(self.a, other.a))
        else: return TCt(max(self.a, other))

    # Отрицание ~
    def __invert__(self): return TCt(1.0 - self.a)

    # Подтверждающее правило
    def Confirm(self, other):
        if isinstance(other, TCt): r = self.a + other.a - self.a*other.a
        else: r = self.a + other - self.a*other
        return TCt(r)

    # Текстовое представление
    def __str__(self): return "{:4.2f}".format(self.a)

    def inc(self, ke):
        self.a = self.a*ke + (1-ke)

    def dec(self, ke):
        self.a = self.a*ke

class TRingBuffer:

    def __init__(self, size):
        self.buff = []
        self.pos = 0
        self.size = size
        for i in range(0, size):
            self.buff.append(0)

    def __lshift__(self, arg):
        if self.pos>=self.size: self.pos = 0
        self.buff[self.pos] = arg
        self.pos += 1
        S = 0
        for e in self.buff:
            if isinstance(e, TCt): S += e.a
            else: S += e
        return TCt(float(S)/float(self.size))

################################################################################

# Подтверждающее правило
def Confirm(v1, v2):
    if isinstance(v1, TCt): res1 = v1.a
    else: res1 = v1
    if isinstance(v2, TCt): res2 = v2.a
    else: res2 = v2
    r = res1 + res2 - res1*res2
    return TCt(r)

def TctStr(e):
    if isinstance(e, TCt):
        s = "{:5.2f}".format(float(e.a))
    else:
        s = "{:5.2f}".format(float(e))
    return s

"""
  einc: ke->0 => f->f, ke->1 => f->1
  edec: ke->0 => f->f, ke->1 => f->0
"""
def einc(a, ke): return a+ke-a*ke  # a*ke + (1.0-ke)

def edec(a, ke): return a*(1.0-ke) # a*ke
