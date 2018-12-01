__author__ = 'sudab'
from gridworld import Gridworld
import numpy as np
from mdp import MDP
from tqdm import tqdm

def get_indices_of_k_smallest(arr, k):
    idx = np.argpartition(arr.ravel(), k)
    return tuple(np.array(np.unravel_index(idx, arr.shape))[:, range(min(k, 0), max(k, 0))])

#gridworld example

nrows = 7
ncols = 5
initial = [3]
moveobstacles = []
targets = [[]]
obstacles = []

regionkeys = {'pavement','gravel','grass','sand','deterministic'}
regions = dict.fromkeys(regionkeys,{-1})
regions['pavement']= range(nrows*ncols)

gwg = Gridworld(initial, nrows, ncols, 1, targets, obstacles,moveobstacles,regions)
gwg.render()
gwg.draw_state_labels()
#
states = range(gwg.nstates)
# alphabet = [0,2] #north, east
alphabet = [0,1,2,3] # south, west
transitions = []
for s in states:
    for a in alphabet:
        for t in np.nonzero(gwg.prob[gwg.actlist[a]][s])[0]:
            p = gwg.prob[gwg.actlist[a]][s][t]
            transitions.append((s, alphabet.index(a), t, p))

mdp = MDP(states, alphabet,transitions)
V, policyT = mdp.max_reach_prob(set(targets[0]), epsilon=0.0001)
V, policyT1 = mdp.max_reach_prob(set([80,81,90,91]), epsilon=0.0001)
agentbehaviours = [policyT,policyT1]
transitions = []
randomness = 0
for ab in agentbehaviours:
    for s in states:
        transdict = dict([(s, agentbehaviours.index(ab), t), 0.0] for t in states)
        for a in range(gwg.nactions):
            if a in ab[s]:
                w = 1.0/len(ab[s]) - randomness/(gwg.nstates - len(ab[s]))
            else:
                w = (1.0 / (gwg.nstates - len(ab[s])))*randomness
            # tempdict = dict([(s, a, t),0.0] for t in states)
            for t in np.nonzero(gwg.prob[gwg.actlist[a]][s])[0]:
                p = gwg.prob[gwg.actlist[a]][s][t]
                transdict[(s, agentbehaviours.index(ab), t)] += p*w
        for t in states:
            transitions.append((s, agentbehaviours.index(ab), t, transdict[(s, agentbehaviours.index(ab), t)]))
mdp1 = MDP(states,alphabet=range(2),transitions=transitions)


R = dict([(s,a,next_s),0.0] for s in mdp.states for a in mdp.available(s) for next_s in mdp.post(s,a) )
R.update([(s,a,next_s),1.0] for s in mdp.states  for a in mdp.available(s) for next_s in mdp.post(s,a) if next_s in targets[0] and s not in targets[0])
V, policyT = mdp.T_step_value_iteration(R,T=20)
policyT1 = dict([s,set(range(gwg.nactions))] for s in mdp.states for a in mdp.available(s))
agentbehaviours = [policyT1,policyT]
transitions = []
for ab in agentbehaviours:
    for s in states:
        transdict = dict([(s, agentbehaviours.index(ab), t), 0.0] for t in states)
        for a in ab[s]:
            # tempdict = dict([(s, a, t),0.0] for t in states)
            for t in np.nonzero(gwg.prob[gwg.actlist[a]][s])[0]:
                p = gwg.prob[gwg.actlist[a]][s][t]
                transdict[(s, agentbehaviours.index(ab), t)] += p*1.0/len(ab[s])
        for t in states:
            transitions.append((s, agentbehaviours.index(ab), t, transdict[(s, agentbehaviours.index(ab), t)]))
mdp2 = MDP(states,alphabet=range(2),transitions=transitions)

mdp1.write_to_file('Class1.txt',initial[0],targets=set(targets[0]))
mdp2.write_to_file('Class2.txt',initial[0],targets=set(targets[0]))

transitions = []
alphabet = {0,1}
accepting_states = set()
file = open('beliefMDP_small.txt', 'r')
print 'Reading MDP'
for line in tqdm(file):
    l = line.split()
    if l[0] != 'b':
        if '|S|' in l[0]:
            states = range(int(l[0][4:]))
        elif l[0]== 'Target':
            l = l[2][1:len(l[2])]
            l = l.split(',')
            accepting_states = set(map(int,l))
        else:
            trans = map(int,l[0:3])
            transitions.append((trans[0],trans[1],trans[2],float(l[3])))


mdpb = MDP(states, alphabet,transitions)
for s in tqdm(states):
    for a in alphabet:
        if a not in mdpb.available(s):
            transitions.append((s,a,s,1))
print 'creating belief MDP'
mdp = MDP(states, alphabet,transitions)
# mdp.write_to_file('truebeliefMDP.txt',initial=initial[0],targets=accepting_states)
print 'Constructing reward matrix for belief MDP'
R = dict([(s,a,next_s),0.0] for s in mdp.states for a in mdp.available(s) for next_s in mdp.post(s,a) )
R.update([(s,a,next_s),1.0] for s in mdp.states  for a in mdp.available(s) for next_s in mdp.post(s,a) if next_s in accepting_states and s not in accepting_states)
# print R
# Solving belief
print('Solving belief MDP')
V,P =  mdp.T_step_value_iteration(R,10)
# print P
# print V

print 'Simulating Belief'
beliefMap = dict()
file = open('beliefStateMapping_small.txt', 'r')
for line in tqdm(file):
    l = line.split(':')
    if '(s,b,c)' not in l[0]:
        ns = int(l[1])
        l = l[0].split(',')
        s = int(l[0])
        c = int(l[2])
        l =  l[1].split('[')
        l = l[1].split(']')[0]
        l = l.split(' ')
        b = map(float,l)
        b[0] = round(b[0],6)
        b[1] = round(1 - b[0],6)
        beliefMap[(s,tuple(b),c)] = ns
# print beliefMap

curr_s = initial[0]
c = 0
curr_b = (0.5,0.5)
groundmdp = mdp1
allmdps = [mdp1,mdp2]
cost_dict = {0:1,1:5}
allbs = np.array([beliefMap.keys()[i][1] for i in range(len(beliefMap))])
indkey = 0
curr_b_s = 0
while True:
    if curr_b_s in accepting_states:
        break
    a = P[curr_b_s].pop()
    print 'Taking action ', a
    next_s = groundmdp.sample(curr_s,a)
    print 'next state ', next_s
    gwg.current = [next_s]
    gwg.render()
    raw_input("Press Enter to continue...")
    print 'Computing new belief'
    c+=cost_dict[a]
    b = [0,0]
    for i in range(2):
        b[i] = allmdps[i].prob_delta(curr_s,a,next_s)*curr_b[i]/(sum([allmdps[j].prob_delta(curr_s,a,next_s)*curr_b[j] for j in range(2) ]))
    b[0] = round(b[0], 6)
    b[1] = round(1 - b[0], 6)
    curr_b = tuple(b)
    bs = np.full((len(beliefMap),2),curr_b)
    k=1000
    indkeyset = get_indices_of_k_smallest(np.linalg.norm(allbs - bs, axis=1),k)[0]
    ind  = 0
    indkey = indkeyset[ind]
    while beliefMap.keys()[indkey] != (curr_s,beliefMap.keys()[indkey][1],c):
        ind+=1
        if ind == k:
            print 'stff'
            break
        indkey = indkeyset[ind]
    print curr_b
    curr_b_s = beliefMap[beliefMap.keys()[indkey]]

print curr_b