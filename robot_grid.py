from gridworld import Gridworld
import numpy as np
from mdp import MDP
import math
from tqdm import tqdm
import itertools
import random
import qlearning
import matplotlib.pyplot as plt

def get_indices_of_k_smallest(arr, k):
    idx = np.argpartition(arr.ravel(), k)
    return tuple(np.array(np.unravel_index(idx, arr.shape))[:, range(min(k, 0), max(k, 0))])

def writeJson(outfile,my_dict=None):
    with open(outfile, 'w') as f:
        [f.write('{0},{1}\n'.format(key, value)) for key, value in my_dict.items()]

num_x = 20
num_y = 15
num_t = 10
max_x = 1200
min_x = -1200
max_y = 1200
min_y = -1200
xrange = [e for e in range(min_x,max_x+1,200)]
yrange =[e for e in range(min_y,max_y+1,200)]
trange = [e for e in range(0,180+1,30)]
alphabet = {'forward','back','left','right','stop','turnleft','turnright','forwardleft','forwardright'}

traterange = [-8,0,8]
xraterange = [-20,0,20]
yraterange = [-20,0,20]
v = 20

actdict = {'right':(0,v,0),
           'left':(0,-v,0),
           'back':(-v,0,0),
           'forward':(v,0,0),
           'stop':(0,0,0),
           'turnleft':(0,0,4),
           'turnright':(0,0,-4),
           'forwardleft':(v,-v,0),
           'forwardright':(v,v,0)
           }

transitions = []
states = []
states2 = []

dt = 10.1

ballpos = (-200,0)
targstates = set()
targ_angle = 90
angle_uncertainty = 0.7
uncertainty_disc = 2
targstates.add((0,0,90))
R = dict()
unsafe_states = set()
successors=dict()
for x in tqdm(xrange):
    for y in yrange:
        for t in trange:
            states2.append((x, y, t))
successors=dict()
for state in states2:
    successors[state]=[]

for x in tqdm(xrange):
    for y in yrange:
        for t in trange:
            states.append((x, y, t))
            if (x, y, t) == (-200, -200, 120):
                asdf = 1
            for action in alphabet:
                trate = actdict[action][2]
                xrate = actdict[action][0]
                yrate = actdict[action][1]
                # if np.sqrt(x**2 + y**2) < 150 and t == targ_angle:
                #     targstates.add((x,y,t))
                    # transitions.append(((x, y, t), action, (x, y, t), 1.0))
                    # R[((x, y, t), action, (x, y, t))] = 0
                if (abs(x) > 1000 or abs(y) > 1000) or (y >= ballpos[1]+100 and abs(x) <= 400) or (t<25 or t>155):
                    transitions.append(((x, y, t), action, (x, y, t), 1.0))
                    R[(x, y, t), action] = -100
                    unsafe_states.add((x,y,t))
                else:
                    if np.sqrt(x ** 2 + y ** 2) < 100 and t == targ_angle:
                        targstates.add((x,y,t))
                        R[(x, y, t), action] = 0
                        transitions.append(((x, y, t), action, (x, y, t), 1.0))
                    transdict = dict([((x, y, t), action, (xnew, ynew, tnew)), 0.0] for xnew in xrange for ynew in yrange for tnew in trange)
                    next_t =  max(min(t + dt*trate,180),0)
                    # next_t = next_t % 360
                    # if action == 'left':
                    #     next_x = max(min(max_x,x+xrate*dt*np.sin(np.radians((next_t+t)/2))),min_x)
                    #     next_y = max(min(max_y,-1*xrate * dt * np.cos(np.radians((next_t + t) / 2))), min_y)


                    next_x = max(min(max_x,x+xrate*dt*np.cos(np.radians((t)))+yrate*dt*np.sin(np.radians((t+t)/2))),min_x)
                    next_y = max(min(max_y,y - yrate*dt*np.cos(np.radians((t))) + xrate*dt*np.sin(np.radians((t+t)/2))),min_y)
                    xs = np.full((len(xrange)), next_x)
                    ys = np.full((len(yrange)), next_y)
                    ts = np.full((len(trange)), next_t)
                    if action != 'turnright'  and action != 'turnleft' and action != 'stop':
                        k = uncertainty_disc
                        angle_uncertainty = 0.7
                    else:
                        k = 1
                        angle_uncertainty = 1
                    indkeysetx = get_indices_of_k_smallest(np.abs(xrange - xs), k)[0]
                    indkeysety = get_indices_of_k_smallest(abs(yrange - ys), k)[0]
                    indkeysett = get_indices_of_k_smallest(abs(trange - ts) - ts, 1)[0]
                    w = []
                    for nx in indkeysetx:
                        for ny in indkeysety:
                            for nt in indkeysett:
                                if nx == indkeysetx[0] and ny == indkeysety[0] and nt == indkeysett[0]:
                                    transdict[((x, y,t), action, (xrange[nx], yrange[ny],trange[nt]))] += angle_uncertainty
                                else:
                                    if action != 'turnright' or action != 'turnleft' or action != 'stop':
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
                                    if (x2,y2,t2) in targstates and transdict[(x,y,t),action,(x2,y2,t2)] > 0.5:
                                        R[(x, y, t), action] = 100
                                        print ((x, y, t), action, (x2, y2, t2))
                                    else:
                                        R[(x, y, t), action] = 0
                                    transitions.append(((x,y,t),action,(x2,y2,t2),transdict[(x,y,t),action,(x2,y2,t2)]))
                                    successors[(x,y,t)].append((x2, y2, t2))

# alphabet = traterange
for action in alphabet:
    R[(0,0,90),action]=100

robot_mdp = MDP(states,alphabet,transitions)
successors=dict()
for state in states:
    successors[state]=dict()




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
# V, policy = robot_mdp.max_reach_prob(targstates,unsafe_states)
# for s in robot_mdp.states:
#     for a in robot_mdp.available(s):
#         for ns in robot_mdp.post(s, a):
#             if s not in targstates and ns in targstates:
#                 R[(s, a, ns)] = 1
#             else:
#                 R[(s, a, ns)] = 0
#V, policy = robot_mdp.E_step_value_iteration(R,unsafe_states,targstates,epsilon=0.7)
#robot_mdp.computeTrace((400,400,60),policy,40,targ = targstates)
#print(policy)
#print(V)
#robot_mdp.policyTofile(policy,'robotpolicyfinestgrid2.txt')



#writeJson('robotpolicy_biggrid2',policy)


def make_epsilon_greedy_policy(Q, epsilon,state,alphabet):
    """
    Creates an epsilon-greedy policy based on a given Q-function and epsilon.

    Args:
        Q: A dictionary that maps from state -> action-values.
            Each value is a numpy array of length nA (see below)
        epsilon: The probability to select a random action . float between 0 and 1.
        nA: Number of actions in the environment.

    Returns:
        A function that takes the observation as an argument and returns
        the probabilities for each action in the form of a numpy array of length nA.

    """
    nA=len(alphabet)
    def policy_fn(state):
        A=dict()
        for act in alphabet:
            A[act]=1*epsilon / (nA*1.0)
        #A = np.ones(nA, dtype=float) * epsilon / nA
        maxval=0
        best_action='left'
        for a in alphabet:
            val=Q[state,a]
            if val>=maxval:
                best_action = a
                maxval=val
        A[best_action] += (1.0 - epsilon)
        return A

    pol=policy_fn(state)
    return pol


def qq_learning(env, num_episodes, num_steps, discount_factor=0.9, alpha=0.9, epsilon=0.1):
    """
    Q-Learning algorithm: Off-policy TD control. Finds the optimal greedy policy
    while following an epsilon-greedy policy

    Args:
        env: OpenAI environment.
        num_episodes: Number of episodes to run for.
        discount_factor: Gamma discount factor.
        alpha: TD learning rate.
        epsilon: Chance the sample a random action. Float betwen 0 and 1.

    Returns:
        A tuple (Q, episode_lengths).
        Q is the optimal action-value function, a dictionary mapping state -> action values.
        stats is an EpisodeStats object with two numpy arrays for episode_lengths and episode_rewards.
    """

    # The final action-value function.
    # A nested dictionary that maps state -> (action -> action-value).

    Q = dict()
    for state in robot_mdp.states:
        for action in alphabet:


            Q[state,action]=0

    # Keeps track of useful statistics
    stats_episode_rewards=np.zeros(num_episodes)

    # The policy we're following
    #policy = make_epsilon_greedy_policy(Q, epsilon,alphabet)

    for i_episode in range(num_episodes):
        # Print out which episode we're on, useful for debugging.
        if (i_episode + 1) % 10 == 0:
            print("Episode number:", i_episode+1)
            #sys.stdout.flush()

        # Reset the environment and pick the first action
        max_x = 1000
        min_x = 400
        max_y = 1000
        min_y = 0
        xrangeinit = [e for e in range(min_x, max_x + 1, 200)]
        yrangeinit = [e for e in range(min_y, max_y + 1, 200)]
        trangeinit = [e for e in range(60, 120 + 1, 30)]
        xinit=random.choice(xrangeinit)
        yinit = random.choice(yrangeinit)
        tinit = random.choice(trangeinit)
        state = (xinit,yinit,tinit)



        # One step in the environment
        # total_reward = 0.0
        for t in range(num_steps):

            # Take a step
            Qpolicy = make_epsilon_greedy_policy(Q, epsilon, state, alphabet)

            #print(Qpolicy)
            #print(type(Qpolicy))

            action_probs = Qpolicy
            rand_val=random.random()
            total=0
            for key in Qpolicy:
                total+=Qpolicy[key]
                if rand_val<=total:
                    action=key
                    break


            #action = np.random.choice(np.arange(len(action_probs)), p=action_probs)
            next_state,target=robot_mdp.computeTrace(state, action, 1, targ=targstates)

            #next_state, reward, done, _ = env.step(action)
          #  next_state=0
            print(next_state)
            rand_val=random.random()




            # Update statistics
            #stats.episode_lengths[i_episode] = t

            # TD Update
            reward= R[(state), action]
            stats_episode_rewards[i_episode] += reward*(discount_factor**(t+1))

            maxval = 0
            for key in Qpolicy:
                val = Qpolicy[key]
                if maxval <= val:
                    best_next_action=key
            #best_next_action = np.argmax(Q[next_state])
            td_target = reward + discount_factor * Q[next_state,best_next_action]
            td_delta = td_target - Q[state,action]
            Q[state,action] += alpha * td_delta

            if t==1000 or reward==-100 or reward==100 or reward==1000:
                print(stats_episode_rewards[i_episode])
                break

            state = next_state

    return Q,stats_episode_rewards

Q, stats = qq_learning(robot_mdp, 10000,100)
#print(Q)
#print(stats)
policy=dict([])
for state in states:
    maxval = 0

    for a in alphabet:
        val = Q[state, a]
        if val >= maxval:
            best_action = a
            maxval = val

    policy[state]=best_action
    (x,y,t)=state
    if (abs(x) > 1000 or abs(y) > 1000) or (y >= ballpos[1] + 100 and abs(x) <= 400) or (t < 25 or t > 155):
        policy[state]='stop'


def qTofile(Qvalues, states,alphabet, outfile):
    file = open(outfile, 'w')
    file.write('Qvalues = dict()\n')
    for s in states:
        for act in alphabet:

            file.write('Qvalues[' + str(s) + ' ,'+str(act)+ '] = ' + str(Qvalues[s,act]) + '\n')

    file.close()

qTofile(Q,states,alphabet,'qvalue.txt')



#robot_mdp.policyTofile(Q,'robotexpectedreward.txt')
robot_mdp.policyTofile(policy,'qlearning_policy.txt')

def moving_average(data_set, periods=3):
    weights = np.ones(periods) / periods
    return np.convolve(data_set, weights, mode='valid')


stats2 = moving_average(np.asarray(stats), 50)

#plt.plot(stats)
plt.plot(stats2)

plt.ylabel('expected reward')
plt.show()