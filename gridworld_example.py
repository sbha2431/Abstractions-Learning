__author__ = 'sudab'
from gridworld import Gridworld
#gridworld example

nrows = 7
ncols = 7
initial = 40
targets = [8]
obstacles = [9,16,24,31,32]

regionkeys = {'pavement','gravel','grass','sand','deterministic'}
regions = dict.fromkeys(regionkeys,{-1})
regions['sand']= range(nrows*ncols)

gwg = Gridworld(initial, nrows, ncols, targets, obstacles,regions)
gwg.render()
gwg.draw_state_labels()

R = dict([(s,a,next_s),0] for s in gwg.mdp.states for a in gwg.mdp.alphabet for next_s in gwg.mdp.post(s,a))
R.update([(s,a,next_s),1] for s in gwg.mdp.states for a in gwg.mdp.alphabet for next_s in gwg.mdp.post(s,a) if next_s in targets and s not in targets)

V,P =  gwg.mdp.max_reach_prob({8})