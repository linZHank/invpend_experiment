#! /usr/bin/env python
import rospy
import random

from cartpole import CartPole

def main():
    print("Initializing node... ")
    rospy.init_node('cart_random_move')
    agent = CartPole()
    rospy.on_shutdown(agent.clean_shutdown)
    for _ in range(100):
        agent.take_action(random.uniform(-10,10))
    rospy.spin()

if __name__ == '__main__':
    main()
