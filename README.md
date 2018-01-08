# Inverted Pendulum Experiment
* Author: linZHank
> Using gazebo to simulate inverted pendulum and control through ROS

> The URDF model of the inverted pendulum and gazebo model spawn were refered to [this tutorial](http://gazebosim.org/tutorials?tut=ros_urdf&cat=connect_ros)

## How to use it
> My configuration was Ubuntu 16.04, ROS-Kinetic and Gazebo\-7.0
1. cd to the `/src` directory in ROS workspace \(e.g. `cd ~/ros_ws/src`\)
2. `git clone https://github.com/linZHank/invpend_experiment.git`
3. `catkin_make`, or `carkin build` if you were using [Catkin Command Line Tools](https://catkin-tools.readthedocs.io/en/latest/)
4. `source ~/ros_ws/devel/setup.bash`
5. run `roslaunch invpend_description invpend_rviz.launch` to check the model in rviz;
   run `roslaunch invpend_control load_invpend.launch` to spawn the model in gazebo and initiate ros_control

## Current Issues:
- \[Fixed\]It seems ros_control was keeping exerting control to the joints, which maintains the pole not falling down. _Set pid to 0, 0, 0 in config yaml file_
- Cannot reset simulation: if "/gazebo/reset_simulation" service called, gazebo model went back to initial, however "ROS time moved backwards" error appeared and cannot get data from all topics. Logs are as follows
```
Traceback (most recent call last):
  File "invpend_control/scripts/vel_ctrl_test.py", line 86, in <module>
    main()
  File "invpend_control/scripts/vel_ctrl_test.py", line 82, in main
    cart.wobble()
  File "invpend_control/scripts/vel_ctrl_test.py", line 59, in wobble
    rate.sleep()
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/timer.py", line 103, in sleep
    sleep(self._remaining(curr_time))
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/timer.py", line 164, in sleep
    raise rospy.exceptions.ROSTimeMovedBackwardsException(time_jump)
rospy.exceptions.ROSTimeMovedBackwardsException: ROS time moved backwards
Shuting dwon...
```