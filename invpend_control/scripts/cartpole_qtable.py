#!/usr/bin/env python
import numpy as np
import math
import random
import time

import rospy
from cartpole import CartPole

# Environment related constants
ACTIONS = (-1, 0, 1) # discrete velocity command
NUM_ACTIONS = len(ACTIONS)
upper_bound = [2.4, 1, math.pi/6, math.radians(50)]
lower_bound = [-2.4, -1, -math.pi/6, -math.radians(50)]
STATE_BOUNDS = zip(lower_bound, upper_bound)
NUM_BUCKETS = (3, 3, 6, 3) # (pos_cart, vel_cart, pos_pole, vel_pole)
# Learning related constants
MIN_LEARNING_RATE = 0.1
MIN_EXPLORE_RATE = 0.01
# Simulation related constans
NUM_EPISODES = 1000
MAX_STEP = 250
STREAK_TO_END = 120
SOLVED_STEP = 199

class QlearnCartPole(CartPole):
    """ Inherent from CartPole Class and add q-learning method """
    def __init__(self):
        CartPole.__init__(self)

    def train(self):
        learning_rate = get_learning_rate(0)
        explore_rate = get_explore_rate(0)
        discount_factor = 0.99  # since the world is unchanging
        num_streaks = 0

        # Q-table
        q_table = np.zeros(NUM_BUCKETS + (NUM_ACTIONS,))

        for episode in range(NUM_EPISODES):
            # initialize environment
            self.resetEnv()
            # observe initial states, ([pos_cart, vel_cart, pos_pole, vel_pole])
            ob, _, _ = self.observeEnv()
            # find out observation's index in q_table (s1,s2,s3,s4,_)
            state_0 = observeToBucket(ob)
            # 
            for step in range(MAX_STEP):
                # select an action
                action = select_action(state_0, explore_rate)
                # execute the action, get new observation
                self.take_action(action)
                # time.sleep(1./50)
                # observe env change
                ob, reward, done = self.observeEnv()
                # convert new observation to bucket
                state = observeToBucket(ob)
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

def get_learning_rate(episode):
    return max(MIN_LEARNING_RATE, min(0.5, 1.0-math.log10((t+1)/25)))

def get_explore_rate(episode):
    return max(MIN_EXPLORE_RATE, min(1, 1.0-math.log10((t+1)/25)))
    
def observeToBucket(state):
    bucket_indice = []
    for i in range(len(state)):
        if state[i] <= STATE_BOUNDS[i][0]:
            bucket_index = 0
        elif state[i] >= STATE_BOUNDS[i][1]:
            bucket_index = NUM_BUCKETS[i] - 1
        else:
            # Mapping the state bounds to the bucket array
            bound_width = STATE_BOUNDS[i][1] - STATE_BOUNDS[i][0]
            offset = (NUM_BUCKETS[i]-1)*STATE_BOUNDS[i][0]/bound_width
            scaling = (NUM_BUCKETS[i]-1)/bound_width
            bucket_index = int(round(scaling*state[i] - offset))
        bucket_indice.append(bucket_index)
    return tuple(bucket_indice)

def select_action(state, explore_rate):
    # Select a random action
    if random.random() < explore_rate:
        action = env.action_space.sample()
    # Select the action with the highest q
    else:
        action = ACTIONS[np.argmax(q_table[state])]
    return action

def main():
    """ Set up Q-learning and run """
    print("Initiating simulation...")
    rospy.init_node('q_learning')
    ql_agent = CartPole()
    rospy.onshutdown(ql_agent.clean_shutdown)
    ql_agent.train()
    rospy.spin()
    
if __name__ == '__main__':
    main()
