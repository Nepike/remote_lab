import sys
import random

from sklearn.preprocessing import normalize
import numpy as np

# TODO: separate problem-specific policies into a separate library


class HierarchicalPolicy(object):
    """
    Policy generalization: uses options (subpolicies that can terminate) insta.
    Policy with a termination condition (may terminate at some point). Policy is a mapping from states (observations)
    to actions.

    Option that always terminates is a simple action (in MDP sense). Options can also use other options inside.
    """
    def __init__(self):
        # options are sequences of actions (usually encoded as policies)
        # option with only one action in it is a regular action
        self.current_option = None # basically, an internal state of the agent renamed for readability

    def termination_function(self, observation):
        return True

    def choose_action(self, observation):
        """
        :param state: current state (in the MDP sense), the last observation or, in general, information to base the
        decision on.
        :return: pair (action, finished). Chosen action, if any (None otherwise) and finished = True if termination has
        occurred.
        """
        # use hierarchical policy (that chooses options)
        if self.current_option is None :
            self.current_option = self.choose_option(observation)

        if self.current_option is None :
            return None, self.termination_function(observation)

        action, option_finished = self.current_option.choose_action(observation)

        if option_finished :
            self.current_option = None

        return action, self.termination_function(observation)

    def choose_option(self, observation):
        '''

        :param observation:
        :return: a hierarchichal policy to execute next
        '''
        return SimpleOption(None)


class FindLeftWall(HierarchicalPolicy):
    def __init__(self):
        super(FindLeftWall, self).__init__()

    def termination_function(self, observation):
        agent_cell = next(filter(lambda cell: '@' in cell, [cell for row in observation.data for cell in row]))

        for wall_x, wall_y in observation.find_all_objects("#"):
            if wall_y == agent_y and wall_x == agent_x - 1:
                return True
        return False

    def choose_action(self, observation):
        return 'left', self.termination_function(observation)


class FindTopWall(HierarchicalPolicy):
    def __init__(self):
        super(FindTopWall, self).__init__()

    def termination_function(self, observation):
        agent_cell = next(filter(lambda cell: '@' in cell, [cell for row in observation.data for cell in row]))

        for wall_x, wall_y in observation.find_all_objects("#"):
            if wall_x == agent_x and wall_y == agent_y + 1:
                return True
        return False

    def choose_action(self, observation):
        return 'up', self.termination_function(observation)


class FindTopLeftCorner(HierarchicalPolicy):
    def __init__(self):
        super(FindTopLeftCorner, self).__init__()

    def termination_function(self, observation):
        if FindTopWall().termination_function(observation) and FindLeftWall().termination_function(observation):
            return True
        else:
            return False

    def choose_option(self, observation):
        if not FindLeftWall().termination_function(observation):
            return FindLeftWall()
        elif not FindTopWall().termination_function(observation):
            return FindTopWall()
        else:
            return None


class GoToObject(HierarchicalPolicy):
    """
    Agent moves to the object in its sight radius. If it doesn't detect it, the option terminates. It also
    terminates if standing on it.
    """
    def __init__(self, symbol, outside=True):
        """

        :param symbol: object to find
        :param outside: if True, ignore object in the nest
        """
        super(GoToObject, self).__init__()

        self.target_symbol = symbol
        self.outside = outside

    def termination_function(self, observation):
        agent_cell = next(filter(lambda cell: '@' in cell, [cell for row in observation.data for cell in row]))

        if self.target_symbol in agent_cell:
            # standing on the object

            # if searching outside the nest and the agent is in the nest then not reached goal yet
            if self.outside and '%' in agent_cell:
                return False
            else:
                return True
        elif any([obj.object_symbol == self.target_symbol for row in observation.data for cell in row for obj in cell]) == 0:
            # object not found, nothing to do
            return True
        else:
            if self.outside:
                found_outside = False
                for coords in observation.find_all_objects(self.target_symbol):
                    if '%' in observation[coords[0]][coords[1]]:
                        continue
                    else:
                        found_outside = True
                        break

                # if found a suitable object then not finished yet
                return not found_outside

            # object detected, moving to it
            return False

    def choose_action(self, observation):
        terminated = self.termination_function(observation)
        if terminated:
            return None, terminated

        agent_x, agent_y = None, None
        for row_i in range(len(observation.data)):
            for col_i in range(len(observation.data[row_i])):
                if any([symbol == '@' for symbol in observation.data[row_i][col_i]]):
                    agent_x, agent_y = row_i, col_i
                    break
            if agent_x is not None and agent_y is not None:
                break

        potential_targets_coords = []
        for row_i in range(len(observation.data)):
            for col_i in range(len(observation.data[row_i])):
                # if searching outside of the nest, ignore objects inside it
                if self.outside and (row_i == agent_x and col_i == agent_y):
                    continue

                if any([symbol == self.target_symbol for symbol in observation.data[row_i][col_i]]):
                    potential_targets_coords.append((row_i, col_i))

        if len(potential_targets_coords) == 0:
            return None, terminated

        # choose closest target
        target_coords = min(potential_targets_coords, key=lambda x: abs(x[0]-agent_x) + abs(x[1]-agent_y))

        if target_coords is None:
            return None, terminated
        else:
            target_x = target_coords[0]
            target_y = target_coords[1]

        action = None
        if agent_x < target_x:
            action = 'right'
        elif agent_x > target_x:
            action = 'left'
        elif agent_y < target_y:
            action = 'up'
        elif agent_y > target_y:
            action = 'down'

        return action, terminated


class ForagePolicy(HierarchicalPolicy):
    def choose_option(self, observation):
        agent_cell = next(filter(lambda cell: '@' in cell, [cell for row in observation.data for cell in row]))
        if observation.carrying_food:
            if '%' in agent_cell:
                # agent at nest
                return SimpleOption('release food')
            else:
                # agent not at nest
                return GoToObject('%', False)
        else:
            if ('*' in agent_cell) and ('%' not in agent_cell):
                # agent on food
                return SimpleOption('grab food')
            else:
                # agent not on food
                return GoToObject('*')

    def termination_function(self, observation):
        """
        :param observation: various parameters including the amount of reward received for bringing food to the nest
        :return: True when got food reward, false otherwise
        """
        if observation.food_reward > 0:
            return True
        else:
            return False


class RescuePolicy(HierarchicalPolicy):
    def choose_option(self, observation):
        agent_cell = next(filter(lambda cell: '@' in cell, [cell for row in observation.data for cell in row]))
        if observation.carrying_ant:
            if '%' in observation[x][y]:
                # agent at nest
                return SimpleOption('release ant')
            else:
                # agent not at nest
                return GoToObject('%', False)
        else:
            if ('&' in observation[x][y]) and ('%' not in observation[x][y]):
                # agent on ant
                return SimpleOption('grab ant')
            else:
                # agent not on ant
                return GoToObject('&')

    def termination_function(self, observation):
        """
        :param observation: various parameters including the amount of reward received for bringing an ant to the nest
        :return: True when got rescue reward, false otherwise
        """
        if observation.rescue_reward > 0:
            return True
        else:
            return False


class RechargePolicy(HierarchicalPolicy):
    """
    Recharges agent until it has a good amount of energy.
    """
    def __init__(self, energy_threshold=50):
        super(RechargePolicy, self).__init__()

        self.ENERGY_THRESHOLD = energy_threshold  # recharge until this amount of energy is stored in the agent

    def choose_option(self, observation):
        agent_cell = next(filter(lambda cell: '@' in cell, [cell for row in observation.data for cell in row]))
        if '%' in observation[x][y]:
            # agent at nest
            if '*' in observation[x][y]:
                # nest has food stored in it
                return SimpleOption('eat food')
            else:
                # TODO: how to handle 'no option' return?
                return None
        else:
            # go to nest
            return GoToObject('%', False)

    def termination_function(self, observation):
        if observation.energy >= self.ENERGY_THRESHOLD:
            return True
        else:
            return False


class SimpleOption(HierarchicalPolicy) :
    """
    Wrapper for a simple one-step action that complies with option (hierarchical policy) interface.
    """
    def __init__(self, action):
        super(SimpleOption, self).__init__()

        self.action = action

    def choose_action(self, observation):
        return self.action, True


class ManualOptionSelection(HierarchicalPolicy) :
    """
    Manual control that allows choosing options.
    """
    def __init__(self):
        super(ManualOptionSelection, self).__init__()

    def choose_option(self, observation):
        while True:
            keys = graphics.wait_for_keys()
            key = keys[0].lower()
            if 'q' == key or 'escape' == key:
                sys.exit(0)
            elif key == 'left':
                return SimpleOption('left')
            elif key == 'right':
                return SimpleOption('right')
            elif key == 'up':
                return SimpleOption('up')
            elif key == 'down':
                return SimpleOption('down')
            elif '1' == key:
                return FindLeftWall()
            elif '2' == key:
                return FindTopLeftCorner()
            elif '3' == key:
                return GoToObject('*')


class RandomGoalPolicy(HierarchicalPolicy):
    def __init__(self):
        super(RandomGoalPolicy, self).__init__()

        self.goals = ['bring food', 'rescue ant', 'recharge']
        self.goal_policies = [ForagePolicy(), RescuePolicy(), RechargePolicy()]
        self.goal_weights = np.ones(len(self.goals))
        self.goal_probabilities = []

        self.set_goal_weights(self.goal_weights)  # translated to probabilities via normalization

    def termination_function(self, observation):
        """
        Never terminate a Goal Policy since it is the core of an agent that drives it to do something.
        """
        return False

    def choose_option(self, observation):
        """
        Randomly chooses a goal from the list of goals and the corresponding policy according to the probability
        distribution.
        :return: chosen goal as a hierarchical policy
        """
        return np.random.choice(self.goal_policies, 1, p=self.goal_probabilities)[0]

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

        self.goal_probabilities = normalize(self.goal_weights[:,np.newaxis], axis=0, norm='l1').ravel()

    def get_params(self):
        return self.get_goal_weights()

    def set_params(self, params):
        self.set_goal_weights(params)


class RandomGoalWithForcedTerminationPolicy(RandomGoalPolicy):
    def __init__(self, no_progress_threshold=5):
        """
        :param no_progress_threshold: number of steps taken without any useful change during a single option before
        terminating it and trying another option.
        """
        super(RandomGoalWithForcedTerminationPolicy, self).__init__()

        self.NO_PROGRESS_THRESHOLD = no_progress_threshold
        self.no_progress_counter = 0

        self.last_observation = None
        self.last_action = None

    def choose_action(self, observation):
        # if (self.last_observation is not None) and (observation == self.last_observation):
        if self.last_action is None:
            self.no_progress_counter += 1
        else:
            self.no_progress_counter = 0

        self.last_observation = observation

        if self.no_progress_counter > self.NO_PROGRESS_THRESHOLD:
            self.current_option = self.choose_option(observation)
            self.no_progress_counter = 0

        action, termination = super(RandomGoalWithForcedTerminationPolicy, self).choose_action(observation)
        self.last_action = action

        return action, termination

