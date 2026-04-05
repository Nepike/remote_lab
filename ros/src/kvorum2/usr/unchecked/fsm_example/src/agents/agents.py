
"""
MIT License

Copyright (c) 2018 Maxim Rovbo

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

"""
TODO: merge agent and hierarchicalpolicy (unify the interface - it is really similar). Maybe use bridge / adapter / proxy pattern
"""

import sys
import random
import copy
import collections

import hierarchichal_policy as policies

from sklearn.preprocessing import normalize
import numpy as np
from scipy import stats


class AgentException(Exception):
    def __init__(self, msg=None):
        super().__init__(msg)


class Agent(object):
    """
    Decision making agent. May or may not have a state, the only requirement is for it to return an action to
    execute based on the observation (information about the problem) provided.
    """
    def __init__(self):
        self.state = None  # internal, not necessarily an MDP state

    def choose_action(self, observation):
        return None


class RandomAgent(Agent):
    def __init__(self, action_space):
        self.action_space = action_space

    def choose_action(self, observation):
        return random.choice(self.action_space)

    def reset(self):
        # nothing to reset, the agent is stateless
        pass


class QLearningAgent(Agent):
    """
    Table-based Q-learning.
    """
    def __init__(self, action_space, eps=0.1, alpha=0.7, gamma=1.):
        self.action_space = action_space

        self.eps = eps  # exploration coefficient (the more it is, the more agent explores)
        self.gamma = gamma  # discount factor
        self.alpha = alpha  # learning rate

        self.last_action = None  # last action chosen by the agent, needed for learning later
        self.last_state = None  # last 'state' the agent was in or the closest thing to a state it has

        self.q_table = {}

    def reset(self):
        """
        Resets agent's parameters to a blank new state, forget all it learned.
        :return:
        """
        self.last_action = None
        self.last_state = None

        self.q_table = {}

    def choose_action(self, observation):
        # can't work with those weird grid-object-arrays, so vectorized to flat number array first
        reward = observation.reward
        state = np.array(observation.vectorize()).tostring()

        # time to do a learning step from previous action (if any), because simulation does not care about
        # the agent learning - the learning method should be called by the agent itself
        self.learn(state, reward)

        # choose action
        if random.uniform(0, 1) < self.eps:
            # exploration by random choice
            action = random.sample(range(len(self.action_space)), 1)[0]
        else:
            # exploitation
            predicted_q_values = [self.predict_q(state, action) for action in range(len(self.action_space))]
            action = np.argmax(predicted_q_values)

        self.last_action = action
        self.last_state = state

        return action

    def learn(self, state, reward):
        if self.last_state is None or self.last_action is None:
            # can't learn without data, probably the first step in episode
            return

        # what we thought we would get
        last_predicted = self.predict_q(self.last_state, self.last_action)

        # our current estimation of what we actually got
        predicted_q_values = [self.predict_q(state, action) for action in range(len(self.action_space))]
        target = reward + self.gamma * self.predict_q(state, np.argmax(predicted_q_values))

        # updated estimation of the usefulness of actions
        self.q_table[self.last_state][self.last_action] = last_predicted + self.alpha * (target - last_predicted)

    def predict_q(self, state, action):
        try:
            q_value = self.q_table[state][action]
        except KeyError:
            # uniform lazy initialization of the table
            self.q_table[state] = np.random.uniform(0, 1., len(self.action_space))
            q_value = self.q_table[state][action]

        return q_value


class SemanticProbabilisticInferenceAgent(Agent):
    def __init__(self, action_space, basic_rule_depth=1, max_plan_length=1, eps=0.1, max_memory=50000):
        self.action_space = range(len(action_space))  # transform to numerical, don't want to work with strings here

        # semantic probabilistic inference table, stores past experience as state-action-new_action entries
        self.spi_table = {}
        self.regularities = []  # spi rules used for decision making and formed based on past experience

        self.eps = eps  # exploration coefficient

        self.basic_rule_depth = basic_rule_depth  # depth of rules-from-rules generation
        self.max_plan_length = max_plan_length  # maximum number of actions to be considered in sequence in rules and experience
        self.max_memory = max_memory  # how many experienced situations to remember (NOT USED currently)

        self.last_observation = None  # mostly for simulation monitoring and debugging
        self.last_action = None
        self.last_state = None

        # None means that agent does not know what action that was (should only be in the first steps of an episode)
        self.last_action_sequence = collections.deque([None] * self.max_plan_length)  # circular buffer via deque

    def reset(self):
        self.spi_table = {}
        self.regularities = []

        self.last_observation = None  # mostly for simulation monitoring and debugging
        self.last_action = None
        self.last_state = None

        self.last_action_sequence = collections.deque([None] * self.max_plan_length)  # circular buffer via deque

    def choose_action(self, observation):
        self.last_observation = observation

        # can't work with those weird grid-object-arrays, so vectorized to flat number array first
        reward = self.discretize_reward(observation.reward)
        state = np.array(observation.vectorize()).tostring()

        # time to do a learning step from previous action (if any), because simulation does not care about
        # the agent learning - the learning method should be called by the agent itself

        # TODO: repair this
        self.learn(state, reward)

        # DEBUG
        #self.preset_rules()

        # choose action
        applicable_rules = self.get_applicable_rules(state)
        if random.uniform(0, 1) < self.eps or not applicable_rules:
            # exploration by random choice
            action = self.choose_random_action()
        else:
            # exploitation
            best_rule = None
            best_performance = None

            for rule in applicable_rules:
                performance = self.get_state_rule_performance(rule, state)

                if performance is None:
                    continue

                if (best_performance is None) or (performance > best_performance):
                    best_performance = performance
                    best_rule = rule

            if best_rule is None or np.allclose([best_performance,], [0.]):
                # if either the best action never had a positive result or it is not found at all, choose random
                action = self.choose_random_action()
            else:
                action = self.get_action_from_rule(best_rule)

        self.last_action = action
        self.last_state = state

        # circular buffer
        self.last_action_sequence.popleft()
        self.last_action_sequence.append(action)

        return action

    def choose_random_action(self):
        return random.sample(range(len(self.action_space)), 1)[0]

    def preset_rules(self):
        """
        For DEBUG. Manually created rules.
        :return:
        """
        self.regularities = [
            # you don't need anything to go forward. Useful when no food nearby, turn via exploration or when tired
            # of only going forward and its expectation drops below the ground.
            [0,
             -1,-1,-1,
             -1,-1,-1,
             -1,-1,-1],

            # look at only ONE thing when food in sight and go get it, tiger
            [0, # go fwd
             -1,-1,-1,
             -1,-1, 2,
             -1,-1,-1],  # food fwd
            [0,
             -1,-1, 2,
             -1,-1,-1,
             -1,-1,-1],
            [0,
             -1,-1,-1,
             -1,-1,-1,
             -1,-1, 2],
            [1,  # left
              2,-1,-1,
             -1,-1,-1,
             -1,-1,-1],
            [1,
             -1, 2,-1,
             -1,-1,-1,
             -1,-1,-1],
            [1,
             -1,-1,-1,
              2,-1,-1,
             -1,-1,-1],
            [2,
             -1,-1,-1,
             -1,-1,-1,
              2,-1,-1],
            [2,
             -1,-1,-1,
             -1,-1,-1,
             -1, 2,-1],
        ]

    def learn(self, state, reward):
        """
        Remember the experience and update the rules.
        """
        if self.last_state is None or self.last_action is None:
            # can't learn without data, probably the first step in episode
            return

        # update number of times this outcome happened
        last_action_sequence = np.array(self.last_action_sequence).tostring()
        if self.last_state in self.spi_table:
            if last_action_sequence in self.spi_table[self.last_state]:
                self.spi_table[self.last_state][last_action_sequence][reward] += 1
            else:
                self.spi_table[self.last_state][last_action_sequence] = {0: 0, 1: 0}
                self.spi_table[self.last_state][last_action_sequence][reward] += 1
        else:
            self.spi_table[self.last_state] = {last_action_sequence: {0: 0, 1: 0}}
            self.spi_table[self.last_state][last_action_sequence][reward] += 1

        # Rules in this version encode in the first element the action number of the precondition,
        # and in the rest - which parts of the state to look at (precondition) and always state that
        # the agent reaches the goal in the postcondition

        # generate rules from rules
        rules_specified_by_state, rules_specified_by_action = self.specify_rule([], len(np.frombuffer(state, dtype=int)))

        rules_prev = copy.copy(rules_specified_by_action)  # add actions without state conditions
        regularities = copy.copy(rules_specified_by_action)
        # regularities = []

        # TODO: check assignments (some variables refer to the same lists and appends happen to rules_prev)
        for k in range(self.basic_rule_depth-1):  # -1 since rules specified by action are already there
            specified_rules = []
            for root_rule in rules_prev:
                # concatenate previous rules with a list of new ones
                rules_specified_by_state, rules_specified_by_action = self.specify_rule(root_rule, len(np.frombuffer(state, dtype=int)))
                if rules_specified_by_state:
                    specified_rules += rules_specified_by_state

                if rules_specified_by_action:
                    specified_rules += rules_specified_by_action

                for specified_rule in specified_rules:
                    if self.check_rule_regularity(specified_rule, root_rule):
                        regularities.append(specified_rule)
            rules_prev = specified_rules

        # TODO: add k > d check for regularities

        # TODO: check regularities against [] rule, too so that only one action rules are not all included
        # TODO: remove duplicates (or rather, properly explore the graph since it is not a tree)
        self.regularities = regularities

    def specify_rule(self, rule, nstates):
        """
        Adds more preconditions to a rule of any possible combinations and returns a list of specified rules.
        :param rule: rule to specify
        :param nstates: number of state variables that define the state / observation
        :return:
        """
        specified_rules_by_state = []
        specified_rules_by_action = []

        if not rule:
            # rule is empty - add at least an action
            for action in range(len(self.action_space)):
                new_rule = [action] + [-1] * (self.max_plan_length - 1 + nstates)
                specified_rules_by_action.append(new_rule)
        else:
            # TODO: 0/2/3 are strong dependencies coming from vectorization of foraging observation, remove it
            # add any state param to the rule (change any -1 to 0/2/3)
            for state_bit_index in range(nstates):
                # the first max_plan_length 'bits' encode action sequence
                if rule[state_bit_index + self.max_plan_length] != -1:
                    # can't add this state since it is already used
                    continue

                # change the state bit to some possible observed value
                for bit in [0, 2, 3]:
                    new_rule = copy.copy(rule)
                    new_rule[state_bit_index + self.max_plan_length] = bit

                    specified_rules_by_state.append(new_rule)

            # add an action to the rule
            for action_bit_index in range(self.max_plan_length):
                if rule[action_bit_index] != -1:
                    # can't add this action since the bit is already used
                    continue

                # change the action bit
                for bit in range(len(self.action_space)):
                    new_rule = copy.copy(rule)
                    new_rule[action_bit_index] = bit

                    specified_rules_by_action.append(new_rule)

                # can't have interrupted action sequences - if there is a space, it must be at the end of the actions
                break

        return specified_rules_by_state, specified_rules_by_action

    def discretize_reward(self, reward):
        return 1 if reward > 0.5 else 0

    def get_applicable_rules(self, state):
        """
        Determine, what rules have their precondition fulfilled in this real state (actions are not compared)
        """
        applicable_rules = []

        for rule in self.regularities:
            if self.check_rule_applicable(rule, state):
                applicable_rules.append(rule)

        return applicable_rules

    def check_rule_applicable(self, rule, state):
        """
        :param rule: tested rule
        :param state: observation
        :return:
        """
        state_matched = all([e_r == - 1 or e_r == e_state for e_r, e_state in zip(rule[self.max_plan_length:], np.frombuffer(state, dtype=int))])
        return state_matched

    def check_rule_applicable_experience(self, rule, state, action_state):
        """
        :param rule: tested rule
        :param state: observation part of the experience
        :param action_state: action part of the experience
        :return:
        """
        state_matched = all(
            [e_r == - 1 or e_r == e_state for e_r, e_state in zip(rule[self.max_plan_length:], state)])
        actions_matched = all(
            [e_r == - 1 or e_r == e_action for e_r, e_action in zip(rule[self.max_plan_length:], action_state)])

        return state_matched and actions_matched

    def check_rule_regularity(self, rule, root_rule):
        """
        TODO: DEBUG - rules should be checked against ALL subsets, including EMPTY and this thing is not a tree
        AND use a proper graph search instead of this adhoc
        :param rule:
        :param root_rule:
        :return:
        """

        rule_performance = self.get_general_rule_performance(rule)
        root_rule_performance = self.get_general_rule_performance(root_rule)

        if rule_performance is None or root_rule_performance is None:
            # can't estimate performance without enough data
            return False

        if rule_performance > root_rule_performance:
            return True
        else:
            return False

    def get_general_rule_performance(self, rule):
        """
        Estimate probability of goal achievement using this rule in general.
        """
        total_hits = 0  # situations where rule was applicable
        total_positive_hits = 0  # situations where rule was applicable and goal was achieved
        for stored_encoded_state, action_reward in self.spi_table.items():
            for stored_encoded_action, reward in action_reward.items():
                # transform encoded string representation of state back into numpy array
                stored_state = np.frombuffer(stored_encoded_state, dtype=int).tolist()
                stored_action = np.frombuffer(stored_encoded_action, dtype=int).tolist()

                if self.check_rule_applicable_experience(rule, stored_state, stored_action):
                    # all positive and zero reward cases
                    total_hits += reward[0] + reward[1]

                    # only actual reward cases
                    total_positive_hits += reward[1]

        if total_hits == 0:
            return None
        else:
            return total_positive_hits / total_hits

    def get_state_rule_performance(self, rule, state):
        """
        TODO: Estimate probability of goal achievement using this rule from this state.
        """
        if state not in self.spi_table:
            return None

        total_hits = 0  # situations where rule was applicable
        total_positive_hits = 0  # situations where rule was applicable and goal was achieved
        for stored_encoded_action, reward in self.spi_table[state].items():
            # transform encoded string representation of state back into numpy array
            stored_action = np.frombuffer(stored_encoded_action, dtype=int).tolist()

            if self.check_rule_applicable_experience(rule, state, stored_action):
                # all positive and zero reward cases
                total_hits += reward[0] + reward[1]

                # only actual reward cases
                total_positive_hits += reward[1]

        if total_hits == 0:
            return None
        else:
            return total_positive_hits / total_hits

    def get_action_from_rule(self, rule):
        """
        TODO: Returns the first action to perform when following the given rule.
        """
        if not rule:
            raise AgentException('Can not get action from an empty rule')

        return rule[0]


class ManualAgent(Agent):
    def __init__(self):
        self.action_space = ['up', 'down', 'left', 'right']

    def choose_action(self, observation):
        action = None
        while not action in self.action_space:
            keys = graphics.wait_for_keys()
            action = keys[0].lower()
            if action == 'escape' or action == 'q':
                sys.exit(0)
            else:
                return action


class OptionAgent(Agent):
    """
    Wrapper to allow HierarchicalPolicy be used as a Agent since there interfaces for choose_action are a bit
    different.
    """
    def __init__(self, hierarchical_policy):
        """
        :param hierarchical_policy: HierarchicalPolicy as an option with termination always set to True as it will
        be ignored
        """
        self.option = hierarchical_policy

    def choose_action(self, observation):
        action, finished = self.option.choose_action(observation)

        return action


class ParametrizedOptionAgent(OptionAgent):
    """
    OptionAgent with parameters.
    """
    def __init__(self, parametrized_policy, initial_params):
        super(ParametrizedOptionAgent, self).__init__(parametrized_policy)
        self.set_params(initial_params)

    def get_params(self):
        return self.option.get_params()

    def set_params(self, params):
        self.option.set_params(params)


def str_to_coords(s):
    return [int(x) for x in s.strip('()').split(',')]

class GoalChooser(object):
    """
    TODO: rename and redo this class thingy
    """
    def __init__(self, goals, weights):
        self.policy = RandomGoalPolicy()

        if len(weights) != len(goals):
            raise Exception('Goals and weights array must be of the same length')

        self.goal_weights = weights
        self.goal_probabilities = []

        self.set_goal_weights(weights)  # update probabilities

        self.goals = goals  # list of goal names to be recognized among actions

    def choose_goal(self, actions, action_names):
        """ choose action to execute from list of possibilities
        actions :: [ (sign_name :: str,
                      action :: Rule,
                      adds :: [LogicForm],
                      rms :: [LogicForm],
                      substs :: [ ( const_or_None, {variables} )  ]
                   ] """
        if len(actions) == 0:
            return None
        else:
            # check allowed goals, set probability to 0 for not available goals
            probabilities = []
            for i in range(len(self.goal_probabilities)):
                if self.goals[i] in action_names:
                    probabilities.append(self.goal_probabilities[i])
                else:
                    probabilities.append(0.)

            probabilities = np.array(probabilities)
            if np.allclose(probabilities, np.zeros_like(probabilities)):
                # if all probabilities are zero, choose at random from all actions (not just goals)
                return random.choice(actions)
            else:
                # choose a goal
                probabilities = normalize(probabilities[:, np.newaxis], axis=0, norm='l1').ravel()

                goal_name = np.random.choice(self.goals, 1, p=probabilities)
                goal_index = action_names.index(goal_name)
                return actions[goal_index]

    def get_goal_weights(self):
        return self.goal_weights

    def set_goal_weights(self, weights):
        """
        Sets weights and probabilities of the goals. Probabilities are normalized weights.
        :param weights: ordered list of weights
        """
        # zero vector can't be normalized properly, so a special check is required
        if np.allclose(weights, np.zeros_like(weights)):
            weights = np.ones_like(weights)

        self.goal_weights = abs(np.array(weights))

        self.goal_probabilities = normalize(self.goal_weights[:, np.newaxis], axis=0, norm='l1').ravel()

    def get_params(self):
        return self.get_goal_weights()

    def set_params(self, params):
        self.set_goal_weights(params)


class ObjectiveFunctionEstimator(object):
    """
    Tracks data relevant to a goal or a performance criterium and calculates the corresponding objective function.

    Since observations and objective functions are problem-specific, this class should be mostly used as a base class.
    """
    def __init__(self):
        # TODO: use a ring buffer or something like it for observations memorization
        self.observations = []  # for objective function computation

    def update(self, observation):
        self.observations.append(observation)

    def get_value(self):
        raise NotImplementedError


class CollectFoodObjective(ObjectiveFunctionEstimator):
    """
    Foraging problem: bring as much food to the nest as possible.
    """
    def get_value(self):
        return sum(o.food_reward for o in self.observations)


class RescueObjective(ObjectiveFunctionEstimator):
    """
    Foraging problem: rescue as many ants as possible.
    """
    def get_value(self):
        return sum(o.rescue_reward for o in self.observations)


class ObjectiveMeasuringAgent(Agent):
    """
    Agent that measures its performance according to an objective function.
    """
    def __init__(self, agent, objective_function):
        """
        :param agent: agent that does the work and is being optimized
        :param objective_function: object that estimates agent's performance which should be maximized
        """
        self.agent = agent
        self.objective_function_estimator = objective_function

    def choose_action(self, observation):
        self.objective_function_estimator.update(observation)

        return self.agent.choose_action(observation)


class OptimizingAgent(ObjectiveMeasuringAgent):
    """
    Takes any agent with parameters and optimizes it for an objective using black-box optimization methods.

    NOTE: if parameters define the policy (state->action function) in a conventional way, this problem can be treated
    as a reinforcement learning one.

    TODO: since adaptation and optimization induce some kind of rational behavior, this agent can be called an 'agent',
    while other agent classes that just choose actions should rather be called animats or something. On the other hand,
    definitions vary.

    TODO: cleanup, maybe separate and rethink optimizing interface and choice
    """
    def __init__(self, agent, objective_function, params=None):
        """
        :param agent: agent that does the work and is being optimized
        :param objective_function: object that estimates agent's performance which should be maximized
        :param params: starting parameters
        """
        super(OptimizingAgent, self).__init__(agent, objective_function, params)

        self.is_optimized = False

    def choose_action(self, observation):
        self.objective_function_estimator.update(observation)

        if self.should_optimize():
            self.params = self.optimize()
            self.agent.set_params(self.params)

        return self.agent.choose_action(observation)

    def should_optimize(self):
        return not self.is_optimized

    def optimize(self):
        """
        TODO: implement some optimization
        :return: optimized parameters
        """
        self.is_optimized = True
        return self.params
