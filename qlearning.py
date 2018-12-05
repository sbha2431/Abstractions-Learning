def q_learning(env, num_episodes, num_steps, discount_factor=0.9, alpha=0.5, epsilon=0.1):
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
            print("Episode number:", i_episode)
            #sys.stdout.flush()

        # Reset the environment and pick the first action
        state = (200,200,60)

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


            #action = np.random.choice(np.arange(len(action_probs)), p=action_probs)
            next_state,target=robot_mdp.computeTrace(state, action, 1, targ=targstates)

            #next_state, reward, done, _ = env.step(action)
          #  next_state=0
            print(next_state)
            rand_val=random.random()




            # Update statistics
            #stats.episode_lengths[i_episode] = t

            # TD Update
            reward= R[((state), action, next_state)]
            stats_episode_rewards[i_episode] += reward

            maxval = 0
            for key in Qpolicy:
                val = Qpolicy[key]
                if maxval <= val:
                    best_next_action=key
            #best_next_action = np.argmax(Q[next_state])
            td_target = reward + discount_factor * Q[next_state,best_next_action]
            td_delta = td_target - Q[state,action]
            Q[state,action] += alpha * td_delta

            if t==1000 or target==True:
                break

            state = next_state

    return Q,stats_episode_rewards