# coding: utf-8
import fsm_alternative as fsm_lib


class FsmRitual(fsm_lib.TAutomaton):
    def __init__(self):
        fsm_lib.TAutomaton.__init__(self, 'Ritual', ['Q2', 'Q1', 'Rtl', 'T', 'PreT', 'Init'], ['T'], 'Init')

        self.add_rule(fsm_lib.TRule('Q1', 'PreT', 'self.steps_counter > self.MAX_ALIVE', 'pass'))
        self.add_rule(fsm_lib.TRule('Q1', 'Q2', 'else', 'self.T = 0'))
        self.add_rule(fsm_lib.TRule('PreT', 'T', 'True', 'self.goStop()'))
        self.add_rule(fsm_lib.TRule('Q2', 'PreT', 'else', 'pass'))
        self.add_rule(fsm_lib.TRule('Q2', 'Q2', 'self.T < self.TRitualTurn', 'self.turnRight(); self.T += 1'))
        self.add_rule(fsm_lib.TRule('Rtl', 'PreT', 'self.steps_counter > self.MAX_ALIVE', 'pass'))
        self.add_rule(fsm_lib.TRule('Q1', 'Q1', 'self.T < self.TRitualTurn', 'self.turnLeft(); self.T += 1'))
        self.add_rule(fsm_lib.TRule('Rtl', 'Q1', 'True', 'self.T = 0'))
        self.add_rule(fsm_lib.TRule('Init', 'Rtl', 'else', 'self.T = 0; self.TRitualTurn = 38; self.MAX_ALIVE = 120;'))
