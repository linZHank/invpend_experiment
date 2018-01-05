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
- It seems ros_control was keeping exerting control to the joints, which maintains the pole not falling down.

