# coding: utf-8

from kvorum2 import fsm_alternative as fsm_lib


class FsmReflex(fsm_lib.TAutomaton):
    def __init__(self):
        fsm_lib.TAutomaton.__init__(self, 'Reflex', ['Init', 'S', '1', '2', '3', 'T'], ['T'], 'Init')

        self.add_rule(fsm_lib.TRule('Init', 'S', 'else', 'self.mytimer = 0; self.MYTIMER1 = 10'))
        self.add_rule(fsm_lib.TRule('S', '1', 'self.fIsLB() and self.fIsRB()', 'self.GoBack(); self.mytimer = 0'))
        self.add_rule(fsm_lib.TRule('S', '1', 'self.fIsLB()', 'self.GoBack(); self.mytimer = 0'))
        self.add_rule(fsm_lib.TRule('S', '2', 'self.fIsRB()', 'self.GoBack(); self.mytimer = 0'))
        self.add_rule(fsm_lib.TRule('S', 'T', 'else', 'self.GoBack(); self.mytimer = 0'))
        self.add_rule(fsm_lib.TRule('1', '1', 'self.mytimer > self.MYTIMER1', 'None'))
        self.add_rule(fsm_lib.TRule('1', '3', 'else', 'self.GoRight(); self.mytimer = 0'))
        self.add_rule(fsm_lib.TRule('2', '2', 'self.mytimer > self.MYTIMER1', 'None'))
        self.add_rule(fsm_lib.TRule('2', '3', 'else', 'self.GoLeft(); self.mytimer = 0'))
        self.add_rule(fsm_lib.TRule('3', '3', 'self.mytimer > self.MYTIMER1', 'None'))
        self.add_rule(fsm_lib.TRule('3', 'T', 'else', 'self.GoBack(); self.mytimer = 0'))
