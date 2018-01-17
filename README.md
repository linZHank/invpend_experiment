# Inverted Pendulum Experiment
* Author: linZHank
> Using gazebo to simulate inverted pendulum and control through ROS

> The URDF model of the inverted pendulum and gazebo model spawn were refered to [this tutorial](http://gazebosim.org/tutorials?tut=ros_urdf&cat=connect_ros)

## How to use it
> My configuration was Ubuntu 16.04, ROS-Kinetic and Gazebo\-7.0. Other combinations of Linux, ROS and Gazebo may work, but not guaranteed.
1. cd to the `/src` directory in ROS workspace \(e.g. `cd ~/ros_ws/src`\)
2. `git clone https://github.com/linZHank/invpend_experiment.git`
3. `catkin_make`, or `carkin build` if you were using [Catkin Command Line Tools](https://catkin-tools.readthedocs.io/en/latest/)
4. `source ~/ros_ws/devel/setup.bash`
5. run `roslaunch invpend_description invpend_rviz.launch` to check the model in rviz;
   run `roslaunch invpend_control load_invpend.launch` to spawn the model in gazebo and initiate ros_control

## Scripts you can play with
All python scripts is located at `/your/path/to/ros_ws/src/invpend_experiment/invpend_control/scripts`.
- `cartpole.py` configures the model to be a reinforcement learning ready enviroment
- `test_env.py` can run some tests on the environment. Choices are sending random command and sinusoidal command to the cart through velocity control
- `qtable.py` runs Q-learning algorithm on the cart-pole, however parameters are undertuned at current stage.

## Demo video
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/ZidldNeV2J0/0.jpg)](https://youtu.be/ZidldNeV2J0)