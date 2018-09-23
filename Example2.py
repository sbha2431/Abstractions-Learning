__author__ = 'sudab'

from mdp import MDP
"""Example problem"""
''
states = {1,2,3,4,5,6,7}
acts = {'N','S','E','W'}
transitions = set() #transitions are: (s,a,s',p)

#Add transitions
transitions.add((1,'N',1,1))
transitions.add((1,'S',4,1))
transitions.add((1,'E',2,1))
transitions.add((1,'W',1,1))

transitions.add((2,'N',2,1))
transitions.add((2,'S',5,1))
transitions.add((2,'E',3,1))
transitions.add((2,'W',1,1))

transitions.add((3,'N',3,1))
transitions.add((3,'S',6,1))
transitions.add((3,'E',3,1))
transitions.add((3,'W',2,1))

transitions.add((4,'N',1,1))
transitions.add((4,'S',4,1))
transitions.add((4,'E',5,1))
transitions.add((4,'W',4,1))

transitions.add((5,'N',2,1))
transitions.add((5,'S',7,1))
transitions.add((5,'E',6,1))
transitions.add((5,'W',4,1))

transitions.add((6,'N',3,1))
transitions.add((6,'S',6,1))
transitions.add((6,'E',6,1))
transitions.add((6,'W',5,1))

transitions.add((7,'N',7,1))
transitions.add((7,'S',7,1))
transitions.add((7,'E',7,1))
transitions.add((7,'W',7,1))

mdp = MDP(states,acts,transitions)
#Define rewards
Rs = dict((s,0) for s in states)
Rs[1] = 0
Rs[2] = -10
Rs[3] = 1
Rs[4] = 0
Rs[5] = 1
Rs[6] = 1
Rs[7] = 2

R = dict(((s,a),Rs[s]) for (s,a,t) in mdp.transitions)

V,policy = mdp.T_step_value_iteration(R)
print V,policy


# ## Abstracted MDP
# states = {1,7}
# acts = {'N','S','E','W'}
# transitions = set() #transitions are: (s,a,s',p)
# transitions.add((1,'N',1,1))
# transitions.add((1,'S',1,0.8))
# transitions.add((1,'S',7,0.2))
# transitions.add((1,'E',1,1))
# transitions.add((1,'W',1,1))
# transitions.add((7,'N',7,1))
# transitions.add((7,'S',7,1))
# transitions.add((7,'E',7,1))
# transitions.add((7,'W',7,1))
# mdp = MDP(states,acts,transitions)
#
# Rs[1] = 1
# Rs[7] = 2
#
# R = dict(((s,a),Rs[s]) for (s,a,t) in mdp.transitions)
# V,policy = mdp.T_step_value_iteration(R)
# print V,policy
#
# ## Abstracted MDP 2
# states = {1,2,3,7}
# acts = {'N','S','E','W'}
# transitions = set() #transitions are: (s,a,s',p)
# transitions.add((1,'N',1,1))
# transitions.add((1,'S',1,1))
# transitions.add((1,'E',2,1))
# transitions.add((1,'W',1,1))
#
# transitions.add((2,'N',2,1))
# transitions.add((2,'S',1,1))
# transitions.add((2,'E',2,1))
# transitions.add((1,'W',1,1))
#
# transitions.add((7,'N',7,1))
# transitions.add((7,'S',7,1))
# transitions.add((7,'E',7,1))
# transitions.add((7,'W',7,1))
# mdp = MDP(states,acts,transitions)
#
# Rs[1] = 1
# Rs[7] = 2
#
# R = dict(((s,a),Rs[s]) for (s,a,t) in mdp.transitions)
# V,policy = mdp.T_step_value_iteration(R)
# print V,policy