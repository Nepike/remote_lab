
class RoamingFsmAgent(object):
    def __init__(self, action_space):
        self.action_space = action_space

    def choose_action(self, observation):
        """
        TODO: Uses a finite state machine to get the next action.

        observation: a description of the agemt's observation about the world.
        returns: action from the action space
        """
        import random
        return random.choice(self.action_space)


    def reset(self):
        """
        Resets agent state as if it was just created (all knowledge is discarded)
        """
        pass

