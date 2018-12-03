from gridworld import Gridworld
import numpy as np
from mdp import MDP
from tqdm import tqdm

def get_indices_of_k_smallest(arr, k):
    idx = np.argpartition(arr.ravel(), k)
    return tuple(np.array(np.unravel_index(idx, arr.shape))[:, range(min(k, 0), max(k, 0))])

def writeJson(outfile,my_dict=None):
    with open(outfile, 'w') as f:
        [f.write('{0},{1}\n'.format(key, value)) for key, value in my_dict.items()]

num_x = 20
num_y = 15
num_t = 10
max_x = 800
min_x = -800
max_y = 800
min_y = -400
xrange = [e for e in range(min_x,max_x+1,100)]
yrange =[e for e in range(min_y,max_y+1,100)]
trange = [e for e in range(0,180+1,10)]
alphabet = {'forward','back','left','right','stop','turnleft','turnright'}

traterange = [-3,0,3]
xraterange = [-25,0,25]
yraterange = [-25,0,25]

actdict = {'forward':(25,0,0),
           'back':(-25,0,0),
           'left':(0,-25,0),
           'right':(0,25,0),
           'stop':(0,0,0),
           'turnleft':(0,0,8),
           'turnright':(0,0,-8),
           'forwardleft':(25,-25,0),
           'forwardright':(25.25,0)
           }

transitions = []
states = []
dt = 4.1
v = 25

ballpos = (0,0)
targstates = set()
targ_angle = 90
angle_uncertainty = 0.7
uncertainty_disc = 2
R = dict()
for x in tqdm(xrange):
    for y in yrange:
        for t in trange:
            states.append((x, y, t))
            for action in alphabet:
                trate = actdict[action][2]
                xrate = actdict[action][0]
                yrate = actdict[action][1]
                # if np.sqrt(x**2 + y**2) < 150 and t == targ_angle:
                #     targstates.add((x,y,t))
                    # transitions.append(((x, y, t), action, (x, y, t), 1.0))
                    # R[((x, y, t), action, (x, y, t))] = 0
                if (abs(x) > 500 or abs(y) > 500) or (y >= ballpos[1] and abs(x) <= 250) or (t<45 or t>135):
                    transitions.append(((x, y, t), action, (x, y, t), 1.0))
                    R[((x, y, t), action, (x, y, t))] = -10
                else:
                    if np.sqrt(x ** 2 + y ** 2) < 150 and t == targ_angle:
                        targstates.add((x,y,t))
                        R[((x, y, t), action, (x, y, t))] = 0
                    transdict = dict([((x, y, t), action, (xnew, ynew, tnew)), 0.0] for xnew in xrange for ynew in yrange for tnew in trange)
                    next_t =  max(min(t + dt*trate,180),0)
                    next_t = next_t % 360
                    next_x = max(min(max_x,x+xrate*dt*np.cos(np.radians((next_t-t)/2)))+yrate*dt*np.sin(np.radians((next_t-t)/2)),min_x)
                    next_y = max(min(max_y,y + yrate*dt*np.cos(np.radians((next_t-t)/2))+xrate*dt*np.sin(np.radians((next_t-t)/2))),min_y)
                    xs = np.full((len(xrange)), next_x)
                    ys = np.full((len(yrange)), next_y)
                    ts = np.full((len(trange)), next_t)
                    if action != 'turnright' != action != 'turnleft' or action != 'stop':
                        k = uncertainty_disc
                    else:
                        k = 1
                    indkeysetx = get_indices_of_k_smallest(np.abs(xrange - xs), uncertainty_disc)[0]
                    indkeysety = get_indices_of_k_smallest(abs(yrange - ys), uncertainty_disc)[0]
                    indkeysett = get_indices_of_k_smallest(abs(trange - ts) - ts, 1)[0]
                    w = []
                    for nx in indkeysetx:
                        for ny in indkeysety:
                            for nt in indkeysett:
                                if nx == indkeysetx[0] and ny == indkeysety[0] and nt == indkeysett[0]:
                                    transdict[((x, y,t), action, (xrange[nx], yrange[ny],trange[nt]))] += angle_uncertainty
                                elif action != 'turnright' or action != 'turnleft' or action != 'stop' :
                                    transdict[((x, y, t), action, (xrange[nx], yrange[ny], trange[nt]))] += (1.0-angle_uncertainty) / (
                                    len(indkeysetx) + len(indkeysety) + len(indkeysett)-2)
                    for x2 in xrange:
                        for y2 in yrange:
                            for t2 in trange:
                                if transdict[(x,y,t),action,(x2,y2,t2)] > 0:
                                #     if abs(x2) > 800 or abs(y2) > 800 or (y2>ballpos[1] and abs(x2) < 500):
                                #         R[((x, y, t), trate, (x2, y2, t2))] = -10
                                #     # elif (x2, y2, t2) == ballpos + tuple({targ_angle}):
                                #     #     R[((x, y, t), trate, (x2, y2, t2))] = 1
                                #     # elif (x, y, t) == ballpos + tuple({targ_angle}):
                                #     #     R[((x, y, t), trate, (x2, y2, t2))] = 0
                                #     else:
                                    R[((x, y, t), action, (x2, y2, t2))] = -1
                                    transitions.append(((x,y,t),action,(x2,y2,t2),transdict[(x,y,t),action,(x2,y2,t2)]))

# alphabet = traterange
robot_mdp = MDP(states,alphabet,transitions)


# R = dict.fromkeys(transdict.keys(),-1)
# for x in tqdm(xrange):
#     for y in yrange:
#         for t in trange:
#             for trate in traterange:
#                 for x2 in xrange:
#                     for y2 in yrange:
#                         for t2 in trange:
#                             if abs(x2) > 800 or abs(y2) > 800:
#                                 R[((x,y,t),trate,(x2,y2,t2))] = -10
#                             elif (x,y,t) == ballpos+tuple({targ_angle}):
#                                 R[((x, y, t), trate, (x2, y2, t2))] = 10

print('Computing policy...')
V, policy = robot_mdp.max_reach_prob(targstates)
# V, policy = robot_mdp.T_step_value_iteration(R,T=10)
print policy
print V
writeJson('robotpolicy_E_fine3',policy)