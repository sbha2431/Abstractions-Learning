__author__ = 'sudab'

from mdp import MDP
import random
import copy
import math

from operator import itemgetter
import abstraction

confidence_parameters = [(0.9, 1.645),
                         (0.95, 1.96),
                         (0.98, 2.326),
                         (0.99, 2.576)]
e = 0.05
d = 0.90

def recompute_epsilon_mix(biggest_prob = 0.999):
    return math.ceil(math.log(e / (3 * 2.0), 2) / math.log(biggest_prob, 2))

def create_abstractMDP(mdp,aggregation):

    states = set(aggregation.keys())
    abstrans = dict()
    abstrans.update({(s,a):set()for s in aggregation.keys() for a in mdp.alphabet})
    for absstate in states:
        for s in aggregation[absstate]:
            for a in mdp.available(s):
                for t in mdp.post(s,a):
                    for s2 in aggregation.keys():
                        if t in aggregation[s2]:
                            abstrans[absstate,a].add(s2)
    abstransprobs = set()
    for (s,a) in abstrans.keys():
        for t in abstrans[(s,a)]:
            abstransprobs.add((s,a,t,1.0/len(abstrans[(s,a)])))

    return abstransprobs



class AbstractMDPLearner(MDP):
    def __init__(self, groundMDP, abstraction):
        # we call the underlying NFA constructor but drop the probabilities
        abstrans = create_abstractMDP(groundMDP, abstraction)
        states = abstraction.keys()
        alphabet = copy.deepcopy(groundMDP.alphabet)
        super(AbstractMDPLearner, self).__init__(states, alphabet, abstrans)
        self.count = {(s, a, t): 0 for (s, a, t, p) in abstrans}
        self.count_sum = {(s, a): 0 for (s, a, t, p) in abstrans}
        self.T = recompute_epsilon_mix()
        self.groundMDP = groundMDP
        self.abstraction = abstraction
        self.observations = {s: [] for s in self.states}

    def groundtoabs(self,s):
        for state in self.abstraction.keys():
            if s in self.abstraction[state]:
                current_abs_state = copy.deepcopy(state)
                return current_abs_state

    def update_count(self, current, action, next_state):
        self.count[(current, action, next_state)] += 1
        self.count_sum[(current, action)] += 1

    def estimator(self):
        alphas = {s: {t: 0 for t in set(self.observations[s])} for s in self.states}
        avars =  {s: {t: 0 for t in set(self.observations[s])} for s in self.states}
        for s in self.states:
            for r in set(self.observations[s]):
                alphas[s][r] = self.observations[s].count(r) / float(len(self.observations[s]))
                avars[s][r] = (self.observations[s].count(r) * (len(self.observations[s]) - self.observations[s].count(r)))\
                    / float(len(self.observations[s])**2*(len(self.observations[s]) + 1))
        return alphas, avars


    def random_exploration(self, initial, n, groundR):
        current = copy.deepcopy(initial)
        for i in range(n):
            flag = 1
            current_abs_state = self.groundtoabs(current)
            action = random.sample(self.available(current_abs_state), 1)[0]
            next_state = self.groundMDP.sample(current, action)
            next_abs_state = self.groundtoabs(next_state)
            self.update_count(current_abs_state, action, next_abs_state)
            self.observations[current_abs_state].append(groundR[current])
            for a in self.available(current):
                if {current} != self.groundMDP.post(current, a):
                    current = copy.deepcopy(next_state)
                    flag = 0
                    break
            if flag == 1:
                current = copy.deepcopy(initial)

    def unknown_exploration(self, current, unknown_states, n, groundR):
        R = dict.fromkeys(self.states,0)
        R.update(dict.fromkeys(unknown_states,1))
        policy = self.T_step_value_iteration(R)
        while True:
            for s in self.abstraction.keys():
                if current in self.abstraction[s]:
                    current_abs_state = copy.deepcopy(s)
            action = policy[current_abs_state]
            next_state = self.groundMDP.sample(current, action)
            next_abs_state = self.abstraction.keys()[self.abstraction.values().index(next_state)]
            self.update_count(current_abs_state, action, next_abs_state)
            self.observations[current_abs_state].add(groundR[current])
            current = copy.deepcopy(next_state)
            if current in unknown_states and self.count_sum[(current, action, next_state)] > n:#self.check_known_trans(current,action,next_state):
                break


    def check_known_trans(self,s , a, t):
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
