learning_rate = get_learning_rate(0)
explore_rate = get_explore_rate(0)
discount_factor = 0.99  # since the world is unchanging
num_streaks = 0

for episode in range(NUM_EPISODES):
    # observe initial states
    ob = observeEnv()
    # convert observation to bucket (s1,s2,s3,s4,a)
    state_0 = observeToBucket(ob)
    # 
    for step in range(MAX_STEP):
        # select an action
        action = select_action(state_0, explore_rate)
        # execute the action, get new observation
        ob, reward, done = take_action(action)
        # convert new observation to bucket
        state = state_to_bucket(ob)
        # update Q-table based on new state
        max_q = np.amax(q_table[state]) # max q-value
        q_table[state_0 + (action,)] += learning_rate*(reward + discount_factor*max_q - q_table[state_0 + (action,)])
        # reset state
        state_0 = state

        # debug, uncomment following prints
        print("\nEpisode = %d" % episode)
        print("step = %d" % step)
        print("Action: %d" % action)
        print("State: %s" % str(state))
        print("Reward: %f" % reward)
        print("Best Q: %f" % max_q)
        print("Explore rate: %f" % explore_rate)
        print("Learning rate: %f" % learning_rate)
        print("Streaks: %d" % num_streaks)

        # finish current episode when max steps reached or cart-pole out of range
        if done:
            print("Episode %d finished after %f time steps" % (episode, step))
            if step >= SOLVED_STEP:
                num_srteaks += 1
            else:
                num_straks = 0
            break
    # finish learning when agent succeeded over certain times consequtively
    if num_streaks > STREAK_TO_END:
        break

    # update parameters
    explore_rate = get_explore_rate(episode)
    learning_rate = get_learning_rate(episode)
            
