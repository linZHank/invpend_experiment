#!/usr/bin/env python


# This script tests if the self-created inverted pendulum can be controled by joint velocity controller


from __future__ import print_function

import math
import random

import rospy
from std_msgs.msg import (UInt16, Float64)
from sensor_msgs.msg import JointState

class Wobbler(object):

    def __init__(self):
        '''
        "Wobbles" cart by commanding joint velocities sinusoidally.
        '''
        self._sub_invpend_states = rospy.Subscriber('/invpend/joint_states', JointState, self.jsCB)
        self._pub_vel_cmd = rospy.Publisher('/invpend/joint1_velocity_controller/command', Float64, queue_size=1)

    def jsCB(self, data):
    	rospy.loginfo("\n~~~Getting Inverted pendulum joint states~~~\n")
    	pos_cart = data.position[0]
    	vel_cart = data.velocity[0]
    	pos_pole = data.position[1]
    	vel_pole = data.velocity[1]

    def wobble(self):
        '''
        Cart performs the wobbling.
        '''
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
	        cart_vel = random.uniform(-15, 15)
        	self._pub_vel_cmd.publish(cart_vel)
        	rate.sleep()


def main():
    """ Joint Velocity Example: Wobbler
    Commands joint velocities of randomly parameterized cosine waves
    to slider_to_cart joint. Demonstrates Joint Velocity Control Mode.
    """
    print("Initializing node... ")
    rospy.init_node('cart_wobble')

    cart = Wobbler()
    cart.wobble()
    rospy.spin()

if __name__ == '__main__':
    main()