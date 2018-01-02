#! /usr/bin/env python

# Implement Q-learning to control cart-pole inverted pendulum
from __future__ import print_function

import numpy as np
import random
import math
from time import sleep

import rospy
from std_srvs.srv import Empty


def reset_invpend_env():
    pass

# Initialize "inverted pendulum" environment
reset_invpend() # need work

# Define states and actions
# Number of discrete states (bucket) per state dimension
NUM_BUCKETS = (1, 1, 6, 3)  # (x, x', theta, theta')
# Number of discrete actions
NUM_ACTIONS = 2 # (left, right)
# Bounds for each discrete state
STATE_BOUNDS = [[-4.8, 4.8], [-0.5, 0.5], [-math.pi, math.pi], [-math.radians(50), math.radians(50)]]
# Index of the action
ACTION_INDEX = len(NUM_BUCKETS)

# Create Q-table
q_table = np.zeros(NUM_BUCKETS + (NUM_ACTIONS,))

# Learning parameters
MIN_EXPLORATION = 0.01
MIN_LEARNING_RATE = 0.1

# Simulation parameters
NUM_EPISODES = 1000
MAX_T = 250
STREAK_TO_END = 120
SOLVED_T = 199
DEBUG_MODE = True


# Simulation Function
def simulate():
    learning_rate = get_learning_rate(0)
    explore_rate = get_explore_rate(0)
    discount_factor = 0.99
    num_streaks = 0

    for episode in range(NUM_EPISODES):
        #reset the inverted pendulum
        reset_invpent() # need work
        obv = subscribe_joint_states_topic() #np array: ([-,-,-,-]), need work
        #initial state
        state_0 = state_to_bucket(obv)

        for t in range(MAX_T):
            #select an action
            action = select_action(state_0, explore_rate)
            #execute action
            obv, reward, done = execute_action(action)
            state = state_to_bucket(obv)
            
            #update Q-table
            best_q = np.amax(q_table[state])
            q_table[state_0 + (action,)] += learning_rate*(reward + discount_factor*(best_q) - q_table[state_0+(action,)])
            #set up for next iteration
            state_0 = state

            # Print data
            if (DEBUG_MODE):
                print("\nEpisode = %d" % episode)
                print("t = %d" % t)
                print("Action: %d" % action)
                print("State: %s" % str(state))
                print("Reward: %f" % reward)
                print("Best Q: %f" % best_q)
                print("Explore rate: %f" % explore_rate)
                print("Learning rate: %f" % learning_rate)
                print("Streaks: %d" % num_streaks)

                print("")

            if done:
                print("Episode %d finished after %f time steps" % (episode, t))
                if (t >= SOLVED_T):
                    num_straks += 1
                else:
                    num_streaks = 0
                break

        #considerdone when solved over 120 times consecutively
        if num_streaks > STREAK_TO_END:
            break
        #update parameters
        explore_rate = get_explore_rate(episode)
        learning_rate = get_learning_rate(episode)


def select_action(state, explore_rate):
    if random.random() < explore_rate:
        # pick a random action
        action = random_pick_action_from_action_pool() # need work
    else:
        # pick an action with highest q value
        action = np.argmax(q_table[state])
    return action

def get_explore_rate(t):
    return max(MIN_EXPLORE_RATE, min(1, 1.0 - math.log10((t+1)/25)))

def get_learning_rate(t):
    return max(MIN_LEARNING_RATE, min(0.5, 1.0 - math.log10(t+1)/25)))

def state_to_bucket(state):
    bucket_indices = []
    for i in range(len(state)):
        if state[i] <= STATE_BOUNDS[i][0]:
            bucket_index = 0
        elif state[i] >= STATE_BOUNDS[i][0]:
            bucket_index = NUM_BUCKETS[i] - 1
        else:
            #mapping the state bounds to the bucket array
            bound_width = STATE_BOUNDS[i][1] - STATE_BOUNDS[i][0]
            offset = (NUM_BUCKETS[i]-1)*STATE_BOUNDS[i][0]/bound_width
            scaling = (NUM_BUCKETS[i]-1)/bound_width
            bucket_index = int(round(scaling*state[i] - offset))
        bucket_indices.append(bucket_index)
    return tuple(bucket_indice)

if __name__ == "__main__":
    simulate()
            
