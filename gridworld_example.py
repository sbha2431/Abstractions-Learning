__author__ = 'sudab'
from gridworld import Gridworld
import numpy as np
from mdp import MDP
#gridworld example

nrows = 3
ncols = 3
initial = [3]
moveobstacles = [4]
targets = [[]]
obstacles = []

regionkeys = {'pavement','gravel','grass','sand','deterministic'}
regions = dict.fromkeys(regionkeys,{-1})
regions['sand']= range(nrows*ncols)

gwg = Gridworld(initial, nrows, ncols, 1, targets, obstacles,moveobstacles,regions)
gwg.render()
gwg.draw_state_labels()
gwg.mdp.available(1)
#
states = range(gwg.nstates)
# alphabet = [0,2] #north, east
alphabet = [1,3] # south, west
transitions = []
for s in states:
    for a in alphabet:
        for t in states:  # np.nonzero(gwg.prob[gwg.actlist[a]][s])[0]:
            p = gwg.prob[gwg.actlist[a]][s][t]
            transitions.append((s, alphabet.index(a), t, p))

mdp1 = MDP(states, alphabet,transitions)

states = range(gwg.nstates)
alphabet = [0,2] #north, east
transitions = []
for s in states:
    for a in alphabet:
        for t in states: #np.nonzero(gwg.prob[gwg.actlist[a]][s])[0]:
            p = gwg.prob[gwg.actlist[a]][s][t]
            transitions.append((s, alphabet.index(a), t, p))

mdp2 = MDP(states, alphabet,transitions)
# mdp.write_to_file('Class2.txt',initial[0])
# a = 1

transitions = []
alphabet = {0,1}
accepting_states = set()
file = open('beliefMDP.txt', 'r')
for line in file:
    l = line.split()
    if l[0] != 'b':
        if '|S|' in l[0]:
            states = range(int(l[0][4:6]))
        elif l[0]== 'Target':
            l = l[2][1:len(l[2])]
            l = l.split(',')
            accepting_states = set(map(int,l))
        else:
            trans = map(int,l[0:3])
            transitions.append((trans[0],trans[1],trans[2],float(l[3])))
mdpb = MDP(states, alphabet,transitions)
for s in states:
    for a in alphabet:
        if a not in mdpb.available(s):
            transitions.append((s,a,s,1))
mdp = MDP(states, alphabet,transitions)

R = dict([(s,a,next_s),0.0] for s in mdp.states for a in mdp.available(s) for next_s in mdp.post(s,a) )
R.update([(s,a,next_s),1.0] for s in mdp.states  for a in mdp.available(s) for next_s in mdp.post(s,a) if next_s in accepting_states and s not in accepting_states)
print R

V,P =  mdp.T_step_value_iteration(R,5)
# print P
# print V


beliefMap = dict()
file = open('beliefStateMapping.txt', 'r')
for line in file:
    l = line.split(':')
    if '(s b c)' not in l[0]:
        ns = int(l[1])
        l = l[0].split(',')
        s = int(l[0])
        c = int(l[2])
        l =  l[1].split('[')
        l = l[1].split(']')[0]
        l = l.split(' ')
        b = map(float,l)
        beliefMap[(s,tuple(b),c)] = ns
print beliefMap

curr_s = initial[0]
c = 0
curr_b = (0.5,0.5)
groundmdp = mdp2
allmdps = [mdp1,mdp2]
while True:
    curr_b_s = beliefMap[(curr_s,curr_b,c)]
    if curr_b_s in accepting_states:
        break
    a = P[curr_b_s].pop()
    next_s = mdp2.sample(curr_s,a)
    c+=1
    b = [0,0]
    for i in range(2):
        b[i] = allmdps[i].prob_delta(curr_s,a,next_s)*curr_b[i]/(sum([allmdps[j].prob_delta(curr_s,a,next_s)*curr_b[j] for j in range(2) ]))
    curr_b = tuple(b)

print curr_b