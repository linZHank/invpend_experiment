#!/usr/bin/env python


# This script tests if the self-created inverted pendulum can be controled by joint velocity controller


from __future__ import print_function

import math
import random

import rospy
from std_msgs.msg import (UInt16, Float64)
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty

    
class Testbed(object):
    """ Testbed, for the pupose of testing cart-pole system """
    def __init__(self):
        self._sub_invpend_states = rospy.Subscriber('/invpend/joint_states', JointState, self.jsCB)
        self._pub_vel_cmd = rospy.Publisher('/invpend/joint1_velocity_controller/command', Float64, queue_size=1)

    def reset_simulation(self):
        ns = "/gazebo/reset_simulation"
        reset_simulation = rospy.ServiceProxy(ns, Empty)

    def jsCB(self, data):
    	rospy.loginfo("\n~~~Getting Inverted pendulum joint states~~~\n")
    	pos_cart = data.position[0]
    	vel_cart = data.velocity[0]
    	pos_pole = data.position[1]
    	vel_pole = data.velocity[1]
        if math.fabs(pos_cart) >= 2.4:
            self.reset_simulation()

    def wobble(self):
        '''
        Cart performs the wobbling.
        '''
        rate = rospy.Rate(50)
        start = rospy.Time.now()
        period_factor = 0.3
        amplitude_factor = 1
        
        while not rospy.is_shutdown():
	        cart_vel = random.uniform(-19, 18)
        	self._pub_vel_cmd.publish(cart_vel)
        	rate.sleep()

    def clean_shutdown(self):
        print("Shuting dwon...")
        self._pub_vel_cmd.publish(0)
        return True

def main():
    """ Perform testing actions provided by Testbed class
    """
    print("Initializing node... ")
    rospy.init_node('cart_wobble')
    cart = Testbed()
    rospy.on_shutdown(cart.clean_shutdown)
    cart.wobble()
    rospy.spin()

if __name__ == '__main__':
    main()
