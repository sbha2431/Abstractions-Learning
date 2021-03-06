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
max_x = 1000
min_x = -1000
max_y = 1000
min_y = -1000
xrange = [e for e in range(min_x,max_x+1,200)]
yrange =[e for e in range(min_y,max_y+1,200)]
trange = [e for e in range(90,270+1,30)]
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
targ_angle = 180
angle_uncertainty = 0.99
uncertainty_disc = 2
targstates.add((0,0,180))
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
            if (x, y, t) == (200, 200, 120):
                asdf = 1

            for action in alphabet:
                trate = actdict[action][2]
                xrate = actdict[action][0]
                yrate = actdict[action][1]
                # if np.sqrt(x**2 + y**2) < 150 and t == targ_angle:
                #     targstates.add((x,y,t))
                    # transitions.append(((x, y, t), action, (x, y, t), 1.0))
                    # R[((x, y, t), action, (x, y, t))] = 0
                if (abs(x) > 900 or abs(y) > 900) or (abs(y) <= 400 and x <= -5) or (t<115 or t>245):
                    transitions.append(((x, y, t), action, (x, y, t), 1.0))
                    R[(x, y, t), action] = -0.0001
                    unsafe_states.add((x,y,t))
                else:
                    if np.sqrt(x ** 2 + y ** 2) < 100 and t == targ_angle:
                        targstates.add((x,y,t))
                        R[(x, y, t), action] = 0
                        transitions.append(((x, y, t), action, (x, y, t), 1.0))
                    transdict = dict([((x, y, t), action, (xnew, ynew, tnew)), 0.0] for xnew in xrange for ynew in yrange for tnew in trange)
                    next_t =  max(min(t + dt*trate,270),90)
                    # next_t = next_t % 360
                    # if action == 'left':
                    #     next_x = max(min(max_x,x+xrate*dt*np.sin(np.radians((next_t+t)/2))),min_x)
                    #     next_y = max(min(max_y,-1*xrate * dt * np.cos(np.radians((next_t + t) / 2))), min_y)

                    if (x, y, t) == (200, 200, 210):
                        asdf = 1
                    if t<=180:
                        next_x = max(min(max_x,x+xrate*dt*np.cos(np.radians((t)))+yrate*dt*np.sin(np.radians((t+t)/2))),min_x)
                        next_y = max(min(max_y,y - yrate*dt*np.cos(np.radians((t))) + xrate*dt*np.sin(np.radians((t+t)/2))),min_y)
                    else:
                        next_x = max(min(max_x,x+xrate*dt*np.cos(np.radians((t)))+yrate*dt*np.sin(np.radians((t+t)/2))),min_x)
                        next_y = max(min(max_y,y + yrate*dt*np.cos(np.radians((t))) - xrate*dt*np.sin(np.radians((t+t)/2))),min_y)
                    xs = np.full((len(xrange)), next_x)
                    ys = np.full((len(yrange)), next_y)
                    ts = np.full((len(trange)), next_t)
                    if action != 'turnright'  and action != 'turnleft' and action != 'stop':
                        k = uncertainty_disc
                        angle_uncertainty = 0.9
                    else:
                        k = 1
                        angle_uncertainty = 1
                    indkeysetx = get_indices_of_k_smallest(np.abs(xrange - xs), k)[0]
                    indkeysety = get_indices_of_k_smallest(np.abs(yrange - ys), k)[0]
                    indkeysett = get_indices_of_k_smallest(np.abs(trange - ts) - ts, 1)[0]
                    w = []
                    for nx in indkeysetx:
                        for ny in indkeysety:
                            for nt in indkeysett:
                                if nx == indkeysetx[0] and ny == indkeysety[0] and nt == indkeysett[0]:
                                    if (x, y, t) == (200, 200, 180):
                                        asdf = 1
                                    transdict[((x, y,t), action, (xrange[nx], yrange[ny],trange[nt]))] += angle_uncertainty

                                else:
                                    if (action != 'turnright' or action != 'turnleft' or action != 'stop') and (len(indkeysetx) * len(indkeysety) * len(indkeysett)>1):
                                        transdict[((x, y, t), action, (xrange[nx], yrange[ny], trange[nt]))] += (1.0-angle_uncertainty) / (
                                        len(indkeysetx) * len(indkeysety) * len(indkeysett)-1)
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
                                    if (x2,y2,t2) in targstates and transdict[(x,y,t),action,(x2,y2,t2)] >0.7:
                                        R[(x, y, t), action] = 100
                                        print ((x, y, t), action, (x2, y2, t2))
                                    else:
                                        R[(x, y, t), action] = 0
                                    transitions.append(((x,y,t),action,(x2,y2,t2),transdict[(x,y,t),action,(x2,y2,t2)]))
                                    successors[(x,y,t)].append((x2, y2, t2))

# alphabet = traterange
for action in alphabet:
    R[(0,0,180),'stop']=1000

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


def make_epsilon_greedy_policy(Q, epsilon,state,alphabet,successors,transdict,unsafe_states):
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
    unsafe_act = set()
    for act in alphabet:
        #if state==(200,0,180):
           # print (state)

        for state2 in robot_mdp.states:
            try:
                transval=robot_mdp.prob_delta(state, act, state2)
                #if state == (200, 0, 180):

                   # print(transval,state,state2,act)
                if state2 in unsafe_states and transval>0.5 and (state not in unsafe_states):

                    unsafe_act.add(act)
                    #if state==(200,0,180):
                     #   print(state,state2,unsafe_act,transval)
            except:
                pass


    nA=0
    for act in alphabet:
        if act not in unsafe_act:
            nA=nA+1

    def policy_fn(state,unsafe_act,nA):
        A=dict()
        for act in alphabet:
            if nA>1 and state!=(0,0,180) and 'stop' not in unsafe_act:

                unsafe_act.add('stop')
                nA=nA-1
        for act in alphabet:
            if act not in unsafe_act:
                A[act]=1*epsilon / (nA*1.0)
            else:
                pass
                #print("unsafe action")
        #A = np.ones(nA, dtype=float) * epsilon / nA
        maxval=-1e8
        best_action=random.choice(list(alphabet))
        for a in alphabet:
            if a not in unsafe_act:
                val=Q[state,a]
                if val>maxval:
                    best_action = a
                    maxval=val
        A[best_action] += (1.0 - epsilon)

        return A

    pol=policy_fn(state,unsafe_act,nA)
    return pol


def qq_learning(env, num_episodes, num_steps,transdict,unsafe_states,discount_factor=0.85, alpha=0.9999, epsilon=0.5):
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
    #Q[(0, 0, 180), 'stop'] = 10000


    # Keeps track of useful statistics
    stats_episode_rewards=np.zeros(num_episodes)
    stats_episode_goals=np.zeros(num_episodes)
    stats_episode_crashes=np.zeros(num_episodes)


    # The policy we're following
    #policy = make_epsilon_greedy_policy(Q, epsilon,alphabet)

    for i_episode in tqdm(range(1,num_episodes)):
        epsilon_pol=epsilon-1.0*epsilon*i_episode/num_episodes
        #print(epsilon_pol)
        # Print out which episode we're on, useful for debugging.
        if (i_episode + 1) % 10 == 0:
            print("Episode number:", i_episode+1)
            #sys.stdout.flush()

        # Reset the environment and pick the first action
        max_x = 800

        min_x = 0
        max_y = 800
        min_y = -800
        xrangeinit = [e for e in range(min_x, max_x + 1, 200)]
        yrangeinit = [e for e in range(min_y, max_y + 1, 200)]
        trangeinit = [e for e in range(120, 240 + 1, 30)]
        xinit=random.choice(xrangeinit)
        yinit = random.choice(yrangeinit)
        tinit = random.choice(trangeinit)


        state = (xinit,yinit,tinit)



        # One step in the environment
        # total_reward = 0.0
        for t in range(num_steps):

            # Take a step
            Qpolicy = make_epsilon_greedy_policy(Q, epsilon_pol, state, alphabet,successors,transdict,unsafe_states)
            if (t+1) % 100 == 0:
                print("Episode length:", t+1 )
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
            if state==(0,0,180):
                action='stop'
                print("target state reached")
            #if state==(200,0,180):
              #  print(state,action,Q[state,'forward'],Qpolicy)


            #action = np.random.choice(np.arange(len(action_probs)), p=action_probs)
            next_state,target=robot_mdp.computeTrace(state, action, 1, targ=targstates)

            #next_state, reward, done, _ = env.step(action)
          #  next_state=0
            #print(state,action,next_state)
            #print(Qpolicy)
            rand_val=random.random()




            # Update statistics
            #stats.episode_lengths[i_episode] = t

            # TD Update
            reward= R[(state), action]
            stats_episode_rewards[i_episode-1] += (reward*(discount_factor**(t+1)))


            maxval = -1e8
            for key in alphabet:
                val=Q[next_state,key]
                if maxval < val:
                    best_next_action=key
                    maxval=val
            #best_next_action = np.argmax(Q[next_state])
            if reward>0:
                1
            if next_state not in unsafe_states:
                td_target = reward + discount_factor * Q[next_state,best_next_action]
                td_delta = td_target - Q[state,action]
                Q[state,action] += alpha * td_delta


            if t==num_steps-1 or state in targstates or state in unsafe_states:
                #print(stats_episode_rewards[i_episode-1])
                stats_episode_goals[i_episode] = stats_episode_goals[i_episode - 1]
                stats_episode_crashes[i_episode] = stats_episode_crashes[i_episode - 1]




               # if state in targstates:
                #    stats_episode_goals[i_episode] += 1

                if state in unsafe_states:
                    stats_episode_crashes[i_episode] += 1


                    stats_episode_rewards[i_episode] = stats_episode_rewards[i_episode - 1]
                    print("unsafe")


                    break
                if state in targstates and action=='stop':
                    stats_episode_goals[i_episode] += 1

                    stats_episode_rewards[i_episode] = stats_episode_rewards[i_episode - 1]

                    break
                if t==num_steps-1:

                    stats_episode_rewards[i_episode] = stats_episode_rewards[i_episode - 1]

                    print("max episode length")
                    break



            state = next_state

    return Q,stats_episode_rewards,stats_episode_goals,stats_episode_crashes
num_eps=100000
Q, stats,stats_goal,stats_crash = qq_learning(robot_mdp,num_eps,1000,transdict,unsafe_states)
#print(Q)
#print(stats)
policy=dict([])
for state in states:
    maxval = -1e8
    best_action = random.choice(list(alphabet))
    for a in alphabet:
        val = Q[state, a]
        if val > maxval and val!=0:
            best_action = a
            maxval = val
    if maxval==0:
        best_action = random.choice(list(alphabet))

    #print(best_action)

    policy[state]=best_action
    (x,y,t)=state
    if (abs(x) > 900 or abs(y) > 900) or (abs(y) <= 400 and x <= -5) or (t<115 or t>245):
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

def rolling_average(a, n=len(a)) :
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1:] / n

def running_mean(x, N):
    out = np.zeros_like(x, dtype=np.float64)
    dim_len = x.shape[0]
    for i in tqdm(range(dim_len)):
        out[i]=x[i]/(i+1)

        #cap indices to min and max indices

        #out[i] = out[i]/(i+1)
    return out
f = plt.figure(1)

stats2 = running_mean(np.asarray(stats), num_eps)

np.savetxt("rewards.txt",np.asarray(stats2),newline="\n")
np.savetxt("targets.txt",np.asarray(stats_goal),newline="\n")
np.savetxt("crashes.txt",np.asarray(stats_crash),newline="\n")

#stats_goal=rolling_average(np.asarray(stats_goal),500)
#statsgoals = moving_average(np.asarray(stats_goal), 50)
#print(stats2)
plt.plot(stats2)
#plt.plot(stats2)

plt.ylabel('expected reward')

g = plt.figure(2)

#plt.plot(stats2)
plt.plot(stats_goal)

plt.ylabel('number of targets')
plt.show()

