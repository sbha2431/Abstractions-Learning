__author__ = 'sudab'

from mdp import MDP

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
    absmdp = MDP(states, mdp.alphabet, abstransprobs)
    return absmdp




