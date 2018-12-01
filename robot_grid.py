from gridworld import Gridworld
import numpy as np
from mdp import MDP
from tqdm import tqdm
def get_indices_of_k_smallest(arr, k):
    idx = np.argpartition(arr.ravel(), k)
    return tuple(np.array(np.unravel_index(idx, arr.shape))[:, range(min(k, 0), max(k, 0))])

num_x = 10
num_y = 10
num_t = 10
max_x = 1000
min_x = -1000
max_y = 1000
min_y = -1000
xrange = np.linspace(min_x,max_x,num_x)
yrange = np.linspace(min_y,max_y,num_y)
trange = np.linspace(0+360.0/num_t,360,num_t)
traterange = [-5,0,5]
transitions = []
states = []
dt = 2
v = 20

ballpos = (0,0)
targ_angle = 96
angle_uncertainty = 0.1
uncertainty_disc = 5
for x in tqdm(xrange):
    for y in yrange:
        for t in trange:
            states.append((x, y, t))
            # poss_t = list(np.linspace(t - angle_uncertainty, t + angle_uncertainty, angle_uncertainty_disc))

            for trate in traterange:
                transdict = dict([((x, y, t), trate, (xnew, ynew,tnew)), 0.0] for xnew in xrange for ynew in yrange for tnew in trange)
                next_t =  t + dt*trate
                next_t = next_t % 360
                next_x = max(min(max_x,x+v*dt*np.cos(np.radians(next_t))),min_x)
                next_y = max(min(max_y,y + v*dt*np.sin(np.radians(next_t))),min_y)
                xs = np.full((len(xrange)), next_x)
                ys = np.full((len(yrange)), next_y)
                ts = np.full((len(trange)), next_t)
                indkeysetx = get_indices_of_k_smallest(np.abs(xrange - xs), uncertainty_disc)[0]
                indkeysety = get_indices_of_k_smallest(abs(yrange - ys), uncertainty_disc)[0]
                indkeysett = get_indices_of_k_smallest(abs(trange - ts) - ts, 1)[0]
                for nx in indkeysetx:
                    for ny in indkeysety:
                        for nt in indkeysett:
                            transdict[((x, y,t), trate, (xrange[nx], yrange[ny],trange[nt]))] += 1.0/(len(indkeysetx)+len(indkeysety)+len(indkeysett))
                for x2 in xrange:
                    for y2 in yrange:
                        for t2 in trange:
                            if transdict[(x,y,t),trate,(x2,y2,t2)] > 0:
                                transitions.append(((x,y,t),trate,(x2,y2,t2),transdict[(x,y,t),trate,(x2,y2,t2)]))


alphabet = traterange
robot_mdp = MDP(states,alphabet,transitions)
V,policy = robot_mdp.max_reach_prob(ballpos+{targ_angle})
print policy
print V