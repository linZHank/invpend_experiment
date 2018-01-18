#!/usr/bin/env python


""" Train inverted pendulum to keep balance using Q Learning """


# Import utilities
from __future__ import print_function
import numpy as np
import math
import random
import time
import datetime
import matplotlib.pyplot as plt
import rospy
import tensorflow as tf
# Import CartPole class from cartpole.py
from cartpole import CartPole, bcolors


# Time the code execution
start_time = time.time()
# Reinforement learning environment related settings
## Discrete actions
ACTIONS = (-5., -4., -3., -2., -1., 0., 1., 2., 3., 4., 5.) # discrete velocity command
NUM_ACTIONS = len(ACTIONS)
## Neural Network settings
INPUT_LAYER_SIZE = 4
HIDDEN_LAYER_SIZE = 128
OUTPUT_LAYER_SIZE = 9
## Simulation related constans
NUM_EPISODES = 2000
MAX_STEP = 250
STREAK_TO_END = 120

class QlearnCartPole(CartPole):
    """ Inherent from CartPole class and add q-learning method """
    def __init__(self):
        CartPole.__init__(self)

    def train(self):
        # Traing settings
        gamma = .99 # discount_factor
        epsilon = .1 # explore_rate
        # make space to store episodic reward accumulation
        accumulated_reward = 0
        reward_list = []
        # neural network settings
        ## The variables below hold all the trainable weights.
        ## also try 1 layer model
        states_node = tf.placeholder(dtype=tf.float32, shape=(1,INPUT_LAYER_SIZE))
        qvalue_node = tf.placeholder(dtype=tf.float32, shape=(1,OUTPUT_LAYER_SIZE))
        w1 = tf.Variable(tf.truncated_normal(shape=[INPUT_LAYER_SIZE, HIDDEN_LAYER_SIZE],
                                             stddev = 0.01, dtype=tf.float32))
        b1 = tf.Variable(tf.constant(value=0.01, shape=[HIDDEN_LAYER_SIZE], dtype=tf.float32))
        w2 = tf.Variable(tf.truncated_normal(shape=[HIDDEN_LAYER_SIZE, OUTPUT_LAYER_SIZE],
                                             stddev = 0.01, dtype=tf.float32))
        ## define the neural network
        def model(input_states):
            hidden = tf.nn.relu(tf.matmul(data, w1) + tf.b1)
            return tf.matmul(hidden, w2) + b2
        ## output
        q_values = model(states_node)
        ## predictions, also try tf.nn.softmax(q_values)
        prediction = tf.argmax(q_values)
        # Loss function, also try tf.reduce_sum(tf.square(qvalue_node - q_values))
        # loss = tf.reduce_mean(tf.nn.sparse_softmax_cross_entropy_with_logits(
        #     labels=qvalue_node, logits=q_values))
        loss = tf.reduce_sum(tf.square(qvalue_node - q_values))
        # L2 regularization for the fully connected parameters.
        regularizers = (tf.nn.l2_loss(w1) + tf.nn.l2_loss(b1) +
                        tf.nn.l2_loss(w2) + tf.nn.l2_loss(b2))
        ## (optional) add the regularization term to the loss, uncomment following line.
        # loss += 5e-4 * regularizers
        ## (optional) learning rate decaying, uncomment following lines
        # learning_rate = tf.train.exponential_decay(
        #     0.1,                # Base learning rate.
        #     batch * BATCH_SIZE,  # Current index into the dataset.
        #     train_size,          # Decay step.
        #     0.95,                # Decay rate.
        #     staircase=True)
        ## optimizer, try MomentumOptimizer, AdamOptimizer, etc
        optimizer = tf.train.GradientDescentOptimizer(learning_rate=0.1).minimize(loss)
        
        with tf.Session() as sess:
            sess.run(tf.global_variables_initializer())
            for epoch in range(num_epochs):
                # reset environment
                self.reset_env()
                # initial joint states
                ob, _, _ = self.observe_env()
                accumulated_reward = 0
                while step < MAX_STEP and not out:
                    action_index, Qvalue = sess.run([predict, q_output], feed_dict={input_state_node:np.array(ob)})
                    # pick an action randomly by exploring
                    if np.random.ran(1) < epsilon:
                        action_index = random.randrange(0,NUM_ACTIONS)
                    action = ACTIONS[action_index]
                    # apply action
                    self.take_action(action)
                    # give environment some time to obtain new observation
                    rate.sleep()
                    # obtain new observation from environment
                    ob, reward, out = self.observe_env()
                    Qvalue_next = sess.run(q_output, feed_dict={input_state_node:np.array(ob)})
                    maxQ_next = np.max(Qvalue_next)
                    targetQ = Qvalue
                    targetQ[0,]









                    
                    action = select_action(action_index, epsilon)
        # map joint states to slot in Q table
        state_0 = observeToBucket(ob)
        # get ready to learn
        rate = rospy.Rate(self.freq)
        while not rospy.is_shutdown():
            # select an action with epsilon greedy, decaying explore_rate
            action_index, action = select_action(q_table, state_0, explore_rate)
            # apply action as velocity comand
            self.take_action(action)
            # give the enviroment some time to obtain new observation
            rate.sleep()

            # map new observation to slot in Q table
            state = observeToBucket(ob)
            # update Q-table
            print(bcolors.OKGREEN, "Q table gets update...", bcolors.ENDC)
            max_q = np.amax(q_table[state])
            q_table[state_0 + (action_index,)] += learning_rate*(reward + discount_factor*max_q - q_table[state_0 + (action_index,)])
            if episode <= NUM_EPISODES and num_streaks < STREAK_TO_END:
                if not out and step <= MAX_STEP:
                    print(bcolors.OKBLUE, "::: Episode {0:d}, Step {1:d}".format(episode, step), bcolors.ENDC)
                    state_0 = state
                    accumulated_reward += 1
                    print("$$$ Accumulated reward in episode {0:d} was {1:d}".format(episode, accumulated_reward))
                    step += 1
                else:
                    if step == MAX_STEP:
                        num_streaks += 1
                    else:
                        num_streaks = 0
                    accumulated_reward += 1
                    print("$$$ Accumulated reward in episode {0:d} was {1:d}".format(episode, accumulated_reward))
                    reward_list.append(accumulated_reward)
                    # reset env for next episode
                    self.reset_env()
                    # back to initial joint states
                    ob, _, _ = self.observe_env()
                    # map joint states to slot in Q table
                    state_0 = observeToBucket(ob)
                    step = 0
                    episode += 1
                    explore_rate = get_explore_rate(episode)
                    learning_rate = get_learning_rate(episode)
                    accumulated_reward = 0
            else:
                # stop sending velocity command
                self.clean_shutdown()
                # save reward list and Q table
                reward_list = np.asarray(reward_list) # convert list to numpy array
                np.save('qtable_storage/reward_list' + datetime.datetime.now().strftime("%y-%m-%d-%H-%M") + '.npy', reward_list)
                np.save('qtable_storage/q_table' + datetime.datetime.now().strftime("%y-%m-%d-%H-%M") + '.npy', q_table)
                end_time = time.time() # time stamp end of code
                print(bcolors.WARNING, "@@@ Training finished...\nTraining time was {:.5f}".format(end_time - start_time), bcolors.ENDC)
                plt.plot(reward_list)
                plt.xlabel('Episode')
                plt.ylabel('Accumulated reward')
                plt.show()
                break
            rate.sleep()

# Useful functions
def select_action(action_index, epsilon):
    # Select a random action
    if random.random(1) < epsilon:
        act_idx = random.randrange(0,NUM_ACTIONS)
        action = ACTIONS[act_idx]
        print("!!! Action selected randomly !!!")
    # Select the action with the highest q
    else:
        act_idx = np.argmax(q_table[state])
        action = ACTIONS[act_idx]
        print("||| Action selected greedily |||")
    return act_idx, action

def main():
    """ Set up Q-learning and run """
    print("Initiating simulation...")
    rospy.init_node('q_learning')
    ql_agent = QlearnCartPole()
    rospy.on_shutdown(ql_agent.clean_shutdown)
    ql_agent.train()
    rospy.spin()
    
if __name__ == '__main__':
    main()
