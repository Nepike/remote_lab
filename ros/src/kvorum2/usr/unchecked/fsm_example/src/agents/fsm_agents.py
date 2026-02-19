# coding: utf-8
import fsm_ritual
import fsm_find_object
import fsm_obstacle_avoidance
import fsm_meta

import random

import logging

logging.getLogger(__name__).addHandler(logging.NullHandler())
logger = logging.getLogger(__name__)


class RoamingFsmAgent(object):
    def __init__(self, robot_class, obstacle_threshold=20, trace=False):
        """
        Args:
            robot_class (class): class that has a mapping of actions to integers
        """
        logger.info('RoamingFsmAgent created')
        self.robot_class = robot_class
        self.trace = trace

        # constants
        self.obstacle_threshold = obstacle_threshold

        # start a never-ending FSM
        self.fsm = self.create_meta_fsm()

        self.observation = None
        self.action = self.robot_class.ACTION_DO_NOTHING

    def choose_action(self, observation):
        """
        TODO: Uses a finite state machine to get the next action.
        TODO: add metafsm

        observation: a description of the agent's observation about the world.
        returns: action from the action space
        """
        # if fsm will not choose any action, this will be chosen instead
        # NOTE: DO_NOTHING does NOT stop agent from doing anything, it
        # just informs the controller that no change in action is required
        self.action = self.robot_class.ACTION_DO_NOTHING

        # update the information that fsm methods will use to decide what to do
        self.observation = observation

        # if the observation is empty, agent can't act (usually means that a sensor update
        # has not been finished yet)
        if self.observation is not None:
            # FSM will act calling its methods that must have been bound to correspoding
            # methods in this agent class. These methods will set the current action here
            self.fsm.step()

        # send the current chosen action (it was set by the fsm)
        return self.action

    def reset(self):
        """
        Resets agent state as if it was just created (all knowledge is discarded)
        """
        self.fsm.reset()

        self.observation = None
        self.action = self.robot_class.ACTION_DO_NOTHING

    def create_meta_fsm(self):
        fsm = fsm_meta.FsmMeta()
        fsm.trace = self.trace

        # other
        fsm.loadSearchFsm = self.create_search_fsm
        fsm.loadRandomWalk = self.create_search_fsm
        fsm.rand = random.randrange

        return fsm

    def create_ritual_fsm(self):
        """
        Makes sure the FSM knows what methods to call
        """
        fsm = fsm_ritual.FsmRitual()
        fsm.trace = self.trace

        # actions
        fsm.goStop = self.action_stop
        fsm.turnLeft = self.action_turn_left
        fsm.turnRight = self.action_turn_right

        return fsm

    def create_search_fsm(self, goal_id = -1):
        """
        Makes sure the FSM knows what methods to call
        """
        fsm = fsm_find_object.FsmSearch()
        fsm.trace = self.trace

        # attributes
        fsm.goal_id = goal_id

        # actions
        fsm.goStop = self.action_stop
        fsm.goFwd = self.action_go_fwd
        fsm.turnLeft = self.action_turn_left
        fsm.turnRight = self.action_turn_right

        # predicates
        fsm.isGLeft = self.is_goal_left
        fsm.isGRight = self.is_goal_right
        fsm.isGFwd = self.is_goal_forward
        fsm.isObst = self.is_obstacle
        fsm.isNearGoal = self.is_near_goal

        # other
        fsm.loadRitualFSM = self.create_ritual_fsm
        fsm.loadObstacleAvoidanceFSM = self.create_obstacle_avoidance_fsm
        fsm.rand = random.randrange

        return fsm

    def create_obstacle_avoidance_fsm(self):
        """
        Makes sure the FSM knows what methods to call
        """
        fsm = fsm_obstacle_avoidance.FsmObstacleAvoidance()
        fsm.trace = self.trace

        # actions
        fsm.goStop = self.action_stop
        fsm.goFwd = self.action_go_fwd
        fsm.goBack = self.action_go_back
        fsm.turnLeft = self.action_turn_left
        fsm.turnRight = self.action_turn_right

        # predicates
        fsm.isObstRight = self.is_obstacle_right
        fsm.isObstLeft = self.is_obstacle_left
        fsm.isObstFwd = self.is_obstacle

        # no sensors to detect this anyway, return False always
        fsm.isObstCloseBehind = lambda : False

        # other
        fsm.rand = random.randrange

        return fsm

    def action_stop(self):
        self.action = self.robot_class.ACTION_STOP

    def action_go_fwd(self):
        self.action = self.robot_class.ACTION_GO_FWD

    def action_go_back(self):
        self.action = self.robot_class.ACTION_GO_BACK

    def action_turn_left(self):
        self.action = self.robot_class.ACTION_TURN_LEFT_10

    def action_turn_right(self):
        self.action = self.robot_class.ACTION_TURN_RIGHT_10

    def is_obstacle_left(self):
        # A bug in the simulation causes US rangers to return 0 when
        # no object detected
        if (self.observation.ground[0] == 0):
            return False

        return self.observation.ground[0] < self.obstacle_threshold

    def is_obstacle_right(self):
        # A bug in the simulation causes US rangers to return 0 when
        # no object detected
        if (self.observation.ground[1] == 0):
            return False

        return self.observation.ground[1] < self.obstacle_threshold

    def is_obstacle(self):
        return self.is_obstacle_left() or self.is_obstacle_right()

    def is_goal_forward(self, goal_id):
        return self.observation.color[0] == goal_id

    def is_near_goal(self, goal_id):
        return self.observation.color[1] == goal_id

    def is_goal_left(self, goal_id):
        return self.observation.color[2] == goal_id

    def is_goal_right(self, goal_id):
        return self.observation.color[3] == goal_id

