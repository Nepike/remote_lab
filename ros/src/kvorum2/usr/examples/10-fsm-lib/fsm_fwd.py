# coding: utf-8

from kvorum2 import fsm_alternative as fsm_lib

class FsmGoFwd(fsm_lib.TAutomaton):
    def __init__(self):
        fsm_lib.TAutomaton.__init__(self, 'GoFwd', ['GoFwd', 'Init', 'T', 'PreT'], ['T'], 'Init')

        self.add_rule(fsm_lib.TRule('Init', 'GoFwd', 'else', 'self.MAX_ALIVE = 50'))
        self.add_rule(fsm_lib.TRule('GoFwd', 'GoFwd', 'else', 'self.goFwd()'))
        self.add_rule(fsm_lib.TRule('PreT', 'T', 'True', 'self.goStop(); self.aexit()'))
        self.add_rule(fsm_lib.TRule('GoFwd', 'PreT', 'self.steps_counter > self.MAX_ALIVE', 'None'))
