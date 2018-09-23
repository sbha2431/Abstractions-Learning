
from nfa import NFA
import numpy as np
import random

class MDP(NFA):
    def __init__(self, states, alphabet, transitions=[]):
        # we call the underlying NFA constructor but drop the probabilities
        trans = [(s, a, t) for s, a, t, p in transitions]
        super(MDP, self).__init__(states, alphabet, trans)
        # in addition to the NFA we need a probabilistic transition
        # function
        self._prob_cache = dict()
        for s, a, t, p in transitions:
            self._prob_cache[(s, a, t)] = p
        self._prepare_post_cache()

    def prob_delta(self, s, a, t):
        return self._prob_cache[(s, a, t)]

    def sample(self, state, action):
        """Sample the next state according to the current state, the action, and
        the transition probability. """
        if action not in self.available(state):
            return None
        # N = len(self.post(state, action))
        prob = []
        for t in self.post(state, action):
            prob.append(self.prob_delta(state, action, t))

        next_state = np.random.choice(list(self.post(state, action)),
                                      1, p=prob)[0]
        # Note that only one element is chosen from the array, which is the
        # output by random.choice
        return next_state

    def set_prob_delta(self, s, a, t, p):
        self._prob_cache[(s, a, t)] = p

    def evaluate_policy_E(self,policy,R, epsilon = 0.001, gamma = 0.9):
        V1 = dict.fromkeys(self.states,0)
        while True:
            e = 0
            V = V1.copy()
            for s in self.states:
                if type(policy[s]) == set:
                    a= random.choice(list(policy[s]))
                else:
                    a=policy[s]
                V1[s]= sum([self.prob_delta(s,a,next_s)*(gamma*V[next_s] + R[s,a]) for next_s in self.post(s,a)])
                e = max(e, abs(V1[s] - V[s]))
            if e < epsilon:
                return V

    def expected_utility(self,a, s, U):
        "The expected utility of doing a in state s, according to the MDP and U."
        return sum([self.prob_delta(s,a,next_s) * U[next_s] for next_s in self.post(s,a)])

    def best_policy(self, U):
        """Given an MDP and a utility function U, determine the best policy,
        as a mapping from state to action."""
        pi = {}
        utility = {s:dict() for s in self.states}
        for s in self.states:
            for a in self.available(s):
                utility[s][a] = self.expected_utility(a,s,U)
            pi[s] = utility[s].keys()[utility[s].values().index(max(utility[s].values()))]
        return pi

    def T_step_value_iteration(self,R, T):
        """Solving an MDP by value iteration for T-step horizon"""
        U1 = dict([(s, 0) for s in self.states])
        self._prepare_post_cache()
        policy = dict([(s, set()) for s in self.states])
        t = T
        while t > 0:
            U = U1.copy()
            delta = 0
            for s in self.states:
                U1[s] = max([sum([self.prob_delta(s,a,s1) * (U[s1] + R[s, a])
                                  for s1 in self.post(s, a)])]
                            for a in self.available(s))[0]
                delta = max(delta, abs(U1[s] - U[s]))
            t = t - 1

        for s in self.states:
            Vmax = dict()
            for a in self.available(s):
                Vmax[a] = [sum([self.prob_delta(s,a,s1) * (U[s1] + R[s, a])
                                for s1 in self.post(s, a)])][0]
            maxV = max(Vmax.values())
            for a in Vmax.keys():
                if Vmax[a] == maxV:
                    policy[s].add(a)
        return U, policy

    def E_step_value_iteration(self,R,
                        epsilon=0.001, gamma=0.9):
        U1 = dict([(s, 0) for s in self.states])
        while True:
            U = U1.copy()
            delta = 0
            for s in self.states:
                U1[s] = max([sum([self.prob_delta(s,a,next_s) * (gamma*U[next_s] + R[s,a,next_s]) for next_s in self.post(s,a)])
                                            for a in self.available(s)])
                delta = max(delta, abs(U1[s] - U[s]))
            if delta < epsilon * (1 - gamma) / gamma:
                 break
        policy = self.best_policy(U)
        return policy




    # def E_step_value_iteration(self,R,
    #                     epsilon=0.0001, gamma=0.9):
    #     policyT = dict([])
    #     Vstate1 = dict([])
    #     Vstate1.update({s: 0 for s in self.states})
    #     e = 1
    #     Q = dict([])
    #     while e > epsilon:
    #         Vstate = Vstate1.copy()
    #         for s in set(self.states):
    #             acts = self.available(s)
    #             optimal = 0
    #             act = None
    #             for a in self.available(s):
    #                 Q[(s, a)] = sum([self.prob_delta(s, a, next_s) *
    #                                  (gamma*Vstate[next_s] + R[s,a])
    #                                  for next_s in self.post(s, a)])
    #                 if Q[(s, a)] >= optimal:
    #                     optimal = Q[(s, a)]
    #                     act = a
    #                 else:
    #                     pass
    #             acts = set([])
    #             for act in self.available(s):
    #                 if Q[(s, act)] == optimal:
    #                     acts.add(act)
    #             Vstate1[s] = optimal
    #             policyT[s] = acts
    #             e = abs(max([Vstate1[s] -
    #                          Vstate[s] for s in self.states]))  # the abs error
    #             # print "iteration: {} and the state
    #             # value is {}".format(t, Vstate1)
    #     return Vstate1, policyT

    def max_reach_prob(self, target,epsilon=0.0001):
        """
        infinite time horizon
        Value iteration: Vstate[s] the maximal probability of hitting the
        target AEC within infinite steps.
        """
        policyT = dict([])
        Vstate1 = dict([])
        Win = target
        NAEC = set(self.states) - Win
        Vstate1.update({s: 1 for s in list(Win)})
        Vstate1.update({s: 0 for s in list(NAEC)})
        policyT.update({s: self.available(s) for s in list(Win)})
        e = 1
        Q = dict([])
        while e > epsilon:
            Vstate = Vstate1.copy()
            for s in set(self.states) - Win:
                acts = self.available(s)
                optimal = 0
                act = None
                for a in self.available(s):
                    Q[(s, a)] = sum([self.prob_delta(s, a, next_s) *
                                     Vstate[next_s]
                                     for next_s in self.post(s, a)])
                    if Q[(s, a)] >= optimal:
                        optimal = Q[(s, a)]
                        act = a
                    else:
                        pass
                acts = set([])
                for act in self.available(s):
                    if Q[(s, act)] == optimal:
                        acts.add(act)
                Vstate1[s] = optimal
                policyT[s] = acts
                e = abs(max([Vstate1[s] -
                             Vstate[s] for s in self.states]))  # the abs error
                # print "iteration: {} and the state
                # value is {}".format(t, Vstate1)
        return Vstate1, policyT


