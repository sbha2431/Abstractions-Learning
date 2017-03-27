__author__ = 'sudab'

from mdp import MDP

def create_abstractMDP(mdp,aggregation):

    abstractstates = dict()
    states = aggregation.keys()
    abstrans = dict()
    abstrans.update({(s,a):set()for s in aggregation.keys() for a in mdp.alphabet})
    for absstate in aggregation.keys():

        for s in aggregation[absstate]:
            for a in mdp.available(s):
                for t in mdp.post(s,a):
                    abspost = aggregation.keys()[aggregation.values().index(t)]
                    abstrans[absstate,a].add(abspost)
    abstransprobs = set()
    for (s,a) in abstrans.keys():
        for t in abstrans[(s,a)]:
            abstransprobs.add((s,a,t,1.0/len(abstrans[(s,a)])))
    return abstransprobs




