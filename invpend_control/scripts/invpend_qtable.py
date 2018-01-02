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
reset_invpend_env() # need work on this part

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
