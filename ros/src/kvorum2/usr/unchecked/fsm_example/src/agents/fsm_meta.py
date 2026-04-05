# coding: utf-8
import fsm_alternative as fsm_lib


class FsmMeta(fsm_lib.TAutomaton):
    def __init__(self):
        fsm_lib.TAutomaton.__init__(self, 'Meta', ['Goal1', 'Meta', 'Goal2', 'RandomWalk', 'Init'], [], 'Init')

        self.add_rule(fsm_lib.TRule('Init', 'Meta', 'True', 'pass', 100))
        self.add_rule(fsm_lib.TRule('Goal2', 'Goal2', 'self.inner_fsm.status == self.inner_fsm.FSM_NOT_FINISHED', 'self.inner_fsm.step()', 100))
        self.add_rule(fsm_lib.TRule('Goal1', 'Goal1', 'self.inner_fsm.status == self.inner_fsm.FSM_NOT_FINISHED', 'self.inner_fsm.step()', 100))
        self.add_rule(fsm_lib.TRule('RandomWalk', 'RandomWalk', 'self.inner_fsm.status == self.inner_fsm.FSM_NOT_FINISHED', 'self.inner_fsm.step()', 100))
        self.add_rule(fsm_lib.TRule('Meta', 'Goal2', 'self.rand(3) == 1', 'self.inner_fsm = self.loadSearchFsm(2)', 100))
        self.add_rule(fsm_lib.TRule('Meta', 'Goal1', 'self.rand(3) == 2', 'self.inner_fsm = self.loadSearchFsm(1)', 100))
        self.add_rule(fsm_lib.TRule('Meta', 'RandomWalk', 'self.rand(3) == 0', 'self.inner_fsm = self.loadRandomWalk()', 100))
        self.add_rule(fsm_lib.TRule('Goal1', 'Meta', 'else', 'pass', 100))
        self.add_rule(fsm_lib.TRule('Goal2', 'Meta', 'else', 'pass', 100))
        self.add_rule(fsm_lib.TRule('RandomWalk', 'Meta', 'else', 'pass', 100))
        self.add_rule(fsm_lib.TRule('Meta', 'Meta', 'else', 'pass', 100))
