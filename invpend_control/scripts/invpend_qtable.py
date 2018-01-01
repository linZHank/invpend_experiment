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
# Refer to tutorial 
