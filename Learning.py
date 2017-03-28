__author__ = 'sudab'

from mdp import MDP
import random
import copy
import math
from operator import itemgetter

confidence_parameters = [(0.9, 1.645),
                         (0.95, 1.96),
                         (0.98, 2.326),
                         (0.99, 2.576)]
e = 0.05
d = 0.90

def recompute_epsilon_mix(biggest_prob=0.999):
    return math.ceil(math.log(e / (3 * 2.0), 2) / math.log(biggest_prob, 2))

class MDP_learner(MDP):
    def __init__(self, states, alphabet, transitions=[]):
        # we call the underlying NFA constructor but drop the probabilities
        super(MDP_learner, self).__init__(states, alphabet, transitions)
        trans = dict((s, a, t) for s, a, t, p in transitions)
        self.count = {(s,a,t):0 for (s,a,t) in trans}
        self.count_sum = {(s,a):0 for (s,a) in trans}
        self.T = recompute_epsilon_mix()

    def update_count(self,current,action,next_state):
        self.count[(current,action,next_state)] += 1
        self.count_sum[(current,action,next_state)] += 1

    def random_exploration(self,current,n):
        for i in range(n):
            action = random.sample(self.available(current),1)
            next_state = self.sample(current,action)
            self.update_count(current,action,next_state)
            current = copy.deepcopy(next_state)

    def unknown_exploration(self,current,unknown_states,n):
        i = 0
        R = dict.fromkeys(self.states,0)
        R.update(dict.fromkeys(unknown_states),1)
        policy = self.T_step_value_iteration(R)
        while i < n:
            action = policy[current]
            next_state = self.sample(current,action)
            self.count[(current,action,next_state)] += 1
            self.count_sum[(current,action,next_state)] += 1
            current = copy.deepcopy(next_state)
            if self.check_known_trans(current,action,next_state):
                break


    def check_known_trans(self, s, a, t):
        # if the count is just too low then we reject
        if self.count[(s, a, t)] < 500:
            return False
        k = min([(x, y) for (x, y) in confidence_parameters if x >= d],
                key=itemgetter(0))[1]
        mean = float(self.count[(s, a, t)]) / self.count_sum[(s, a)]
        var = self.count[(s, a, t)] * (self.count_sum[(s, a)] -
                                  self.count[(s, a, t)]) /\
            (math.pow(self.count_sum[(s, a)], 2) * (self.count_sum[(s, a)] + 1))
        dev = math.sqrt(var)
        if (dev * k) / math.sqrt(self.count_sum[(s, a)]) <= e / (3 * len(self.states) * self.T):
            return True
        else:
            return False
