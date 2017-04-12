from mdp import MDP
import numpy as np
import Learning
"""Example problem"""
''
states = {0,1,2,3,4,5}
acts = {'a','b'}
transitions = set() #transitions are: (s,a,s',p)

#Add transitions
transitions.add((0,'a',5,0.5))
transitions.add((0,'a',1,0.5))
transitions.add((0,'b',1,1))

transitions.add((5,'a',5,1))
transitions.add((5,'b',5,1))

transitions.add((1,'a',2,1))
transitions.add((1,'b',2,1))

transitions.add((2,'a',2,0.75))
transitions.add((2,'a',3,0.25))
transitions.add((2,'b',4,1))

transitions.add((3,'a',4,0.01))
transitions.add((3,'a',2,0.99))
transitions.add((3,'b',4,0.01))
transitions.add((3,'b',2,0.99))

transitions.add((4,'a',4,1))
transitions.add((4,'b',4,1))

mdp = MDP(states,acts,transitions)

#Define rewards
Rs = dict((s,0) for s in states)
Rs[0] = 0
Rs[1] = -3
Rs[2] = -2
Rs[3] = -10
Rs[4] = 0
Rs[5] = -1
R = dict(((s,a),0) for (s,a,t) in mdp.transitions)
policy = mdp.T_step_value_iteration(R)
print policy
# # for s in mdp.states:
# #     for a in mdp.available(s):
# #         for t in mdp.post(s,a):
# #             R[(s,a)] = Rs[t]
#
# # abstraction
# aggregation = dict()
# aggregation[0] = {0}
# aggregation[1] = {1,2,3}
# aggregation[2] = {4,5}
#
# AbsLearnMDP = Learning.AbstractMDPLearner(mdp, aggregation)
# initial = 0
# AbsLearnMDP.random_exploration(initial, 10000, Rs)
# alphas, avars = AbsLearnMDP.estimator()
# aprod = {s: [] for s in AbsLearnMDP.states}
# for s in aprod.keys():
#     aprod[s] = [a*b for a, b in zip(alphas[s].keys(), alphas[s].values())]
#     print 'Variance in alpha values in state ', str(s), ' is ', str(np.var(aprod[s]))
#
# print alphas
# print avars
# print aprod
# print AbsLearnMDP.count_sum