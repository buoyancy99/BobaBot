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



## Elevater Utils

Requirements:

-cv2, cv_bridge

### Elevator door detection

This use optic flow for detecting which elevator door opens. Robot must stay still to use, with 2 elevator doors in the frame.

To run:

```bash
rosrun elevator_utils elevator_detection
```

To get the value. 

0: no open door

1: left door open

2: right door open

```python
def door_detection_callback(door_value):
  ...
  
rospy.Subscriber("elevator_door_open", Int8, door_detection_callback)
```

Todo:

Eliminate human movement through human detections

### Elevator floor detection

This uses the features from floor 7 and floor 2 to detect floors

To run:

```bash
rosrun elevator_utils floor_detection
```

To get the value. 

For each floor, binary int for the detection of floor

```python
pub = rospy.Publisher('floor_to_detect', Int8, queue_size=10)
pub.publish(0) #modify this value
# 0: no floor
# 1: floor2 when in the left elevator
# 2: floor2 when in the right elevator
# 3: floor7 when in the left elevator
# 4: floor7 when in the right elevator (direction facing the elevator from outside)

def floor_detection_callback(door_value):
  #0 or 1 for the floor
	return
  
  
rospy.Subscriber("floor_value", Int8, floor_detection_callback)
```




##Speech

Packages Needed: In requirements.txt inside voice_utils

###Elevator Speech
Use to call human to press button:
Use rosrun voice_info Trial.py
If human found inside elevator:
Use rosrun voice_info Trial2.py

###Speech Recognition Part

Voice recognition stuff is in recognizer.py which is written as a ROS service such that it can read input from mic and publish back to the node.

rosrun voice_info recog.py
