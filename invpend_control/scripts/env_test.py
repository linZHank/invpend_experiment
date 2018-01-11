#! /usr/bin/env python
import rospy

from cartpole import CartPole

def main():
    print("Initializing node... ")
    rospy.init_node('cart_random_move')
    agent = CartPole()
    rospy.on_shutdown(agent.clean_shutdown)
    for _ in range(100):
        agent.take_action(1)
    rospy.spin()

if __name__ == '__main__':
    main()
