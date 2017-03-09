from mdp import MDP
from itertools import product
"""Example problem"""
''
states = {0,1,2,3,4,5}
acts = {'a','b'}
transitions = set() #transitions are: (s,a,s',p)

"""Add transitions"""
transitions.add((0,'a',5,0.5))
transitions.add((0,'b',1,0.5))
transitions.add((0,'b',1,1))

transitions.add((5,'a',5,1))
transitions.add((5,'b',5,1))

transitions.add((1,'a',3,1))
transitions.add((1,'b',3,1))

transitions.add((2,'a',2,0.9))
transitions.add((2,'a',3,0.1))
transitions.add((2,'b',4,1))

transitions.add((3,'a',4,0.01))
transitions.add((3,'a',2,0.99))
transitions.add((3,'b',4,0.01))
transitions.add((3,'b',2,0.99))

transitions.add((4,'a',4,1))
transitions.add((4,'b',4,1))

mdp = MDP(states,set(),acts,transitions)

stateaction = set(product(states,acts))
R = dict.fromkeys(stateaction)