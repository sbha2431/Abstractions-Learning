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
V,P =  gwg.mdp.max_reach_prob({8})
print P
print V

