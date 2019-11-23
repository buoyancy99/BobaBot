# EECS106A Project
## RoboRTS Dependency
*sudo apt-get install ros-melodic-libg2o libgoogle-glog-dev

## Installation
First you need to install rosmelodic on ubuntu 18.04
*install intel opencl https://github.com/intel/compute-runtime/releases
*install openvino then ros_openvino here https://github.com/gbr1/ros_openvino

Before making, you'll have to source `setupvars.sh`
```
source /opt/intel/openvino/bin/setupvars.sh
```
The path to your setupvars.sh may be different.

*install camera files 
```
git clone https://github.com/slightech/MYNT-EYE-D-SDK.git ~/Libraries/MYNT-EYE-D-SDK
cd ~/Libraries/MYNT-EYE-D-SDK
make init
make all
sudo make install
```
if there is error about header not found
```
cp ~/MYNT-EYE-D-SDK/wrappers/ros/devel/include/mynteye_wrapper_d/*  
~/PATH_TO_BOBABOT/devel/include/mynteye_wrapper_d
```

## Navigation
http://wiki.ros.org/navigation/Tutorials/RobotSetup#Navigation_Stack_Setup
Dependencies: sudo apt install ros-melodic-navigation
For teleop: `git clone https://github.com/lrse/ros-keyboard.git`
Package name : navigation  
Launch file : robot_activate.launch and robot_nav.launch (for simulation with stage, just robot_nav_sim.launch)  
Align the robot with 2D pose estimate and set the goal with 2D set goal, also can set goal with http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals  
Launch robot_activate.launch first and then robot_nav.launch

