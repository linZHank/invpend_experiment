#!/usr/bin/env python


# This script tests if the self-created inverted pendulum can be controled by joint velocity controller


from __future__ import print_function

import numpy as np
import math
import random
import time

import rospy
from std_msgs.msg import (UInt16, Float64)
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from gazebo_msgs.msg import LinkState
from geometry_msgs.msg import Point

def exceedRange(pos_cart, pos_pole):
    return math.fabs(pos_cart) > 2.4 or math.fabs(pos_pole) > math.pi/2 # cart: +-2.4; pole: +-15degrees

class CartPole(object):
    """ Testbed, for the pupose of testing cart-pole system """
    def __init__(self):
        # init topics and services
        self._sub_invpend_states = rospy.Subscriber('/invpend/joint_states', JointState, self.jsCB)
        self._pub_vel_cmd = rospy.Publisher('/invpend/joint1_velocity_controller/command', Float64, queue_size=1)
        self._pub_set_pole = rospy.Publisher('/gazebo/set_link_state', LinkState)
        self.reset_sim = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        # init parameters
        self.reset_dur = 1 # reset duration, sec
        self.freq = 50 # topics pub and sub frequency, Hz
        self.pos_cart = 0
        self.vel_cart = 0
        self.pos_pole = 0
        self.vel_pole = 0
        self.PoleState = LinkState()
        self.PoleState.link_name = 'pole'
        self.PoleState.pose.position = Point(0.0, -0.25, 2.0)
        self.PoleState.reference_frame = 'world'
        self.ex_rng = False # cart-pole exceed range of mation
        self.cmd = 0

    def jsCB(self, data):
        """ Callback function for subscribing /invpend/joint_states topic """
    	rospy.loginfo("~~~Getting Inverted pendulum joint states~~~")
    	self.pos_cart = data.position[1]
    	self.vel_cart = data.velocity[1]
    	self.pos_pole = data.position[0]
    	self.vel_pole = data.velocity[0]
        # For debug purpose, uncomment the following line
        print("cart_position: {0:.5f}, cart_velocity: {1:.5f}, pole_angle: {2:.5f}, pole_angular_velocity: {3:.5f} ".format(self.pos_cart, self.vel_cart, self.pos_pole, self.vel_pole))
        self.ex_rng = exceedRange(self.pos_cart, self.pos_pole)
        # if self.ex_rng == True:
        #     self.resetEnv()

    def resetEnv(self):
        reset_count = 0
        print("=== reset invpend pos ===\n")
        while reset_count < self.reset_dur*self.freq:
            print("reset counter: ", str(reset_count)) # debug log
            self._pub_vel_cmd.publish(0)
            self._pub_set_pole.publish(self.PoleState)
            reset_count += 1
            rospy.sleep(1./self.freq)
            self.pos_cart = 0
            self.vel_cart = 0
            self.pos_pole = 0
            self.vel_pole = 0
            self.reward = 0
                
    def observeEnv(self):
        """ Get cart-pole state, reward and out of range flag from environment """
        return np.array([self.pos_cart, self.vel_cart, self.pos_pole, self.vel_pole]), self.reward, self.ex_rng

    def take_action(self, vel_cmd):
        rate = rospy.Rate(self.freq)
        while not rospy.is_shutdown():
            if self.ex_rng == True:
                print("cart-pole is out of range") # debug
                self.cmd = 0
            else:
                print("cart-pole is within the range") # debug
                self.cmd = vel_cmd
            self._pub_vel_cmd.publish(self.cmd)
            print("---> publishing velocity command: {:.4f}".format(self.cmd))
            rate.sleep()

    def clean_shutdown(self):
        print("Shuting down...")
        self._pub_vel_cmd.publish(0)
        return True
