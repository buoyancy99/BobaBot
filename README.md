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

TODO: Add launch files for what to do inside elevator (7 to 2, 2 to 7), entering/exiting floor 2, exiting 7. Edit `run_integrated.py` correspondingly. Edit `SDH7_enter_elevator.py` to receive "ding" input. \
NOTE: `run_integrated_sim` and `robot_nav_sim.launch` currently may not work without some editing. If needed, ask Alex for help.

Package name : navigation \
http://wiki.ros.org/navigation/Tutorials/RobotSetup#Navigation_Stack_Setup \
Dependencies: sudo apt install ros-melodic-navigation \
For teleop: `git clone https://github.com/lrse/ros-keyboard.git` \
Launch file : robot_nav.launch (for simulation with stage, just robot_nav_sim.launch)  \
Aligns the robot with 2D pose estimate and sets the goal with 2D set goal. Can also set goal with http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals

To run:
``` bash
roslaunch navigation robot_nav.launch
```

### Code overview

`robot_nav.launch`: Loads `SDH_all_floorplan` (found in `maps`) into `map_server`. Launches `amcl`, `move_base`, and `rviz`. Runs `run_integrated.py` (found in `scripts`), which has the high-level logic for operation.

`run_integrated.py`: Broad overview: sets initial poses and goals for `move_base`. Uses the `roslaunch` library for python to launch the elevator logic. In particular, launches `SDH7_enter_elevator.launch` and kills upon receiving a callback that elevator operation is done. \
Subscriptions: `/elevator/done`: Upon receiving a message, kills the latest launch file.

`SDH7_enter_elevator.launch`: Runs `position_controller.py`, `SDH7_enter_elevator.py` (both in `scripts`), `lidar_elevator_detection.py` (in `elevator_utils` package). 

`position_controller.py`: Gives commands to the robot given waypoints. Does basic collision avoidance (stopping). Used exclusively for elevator logic. `move_base` controls all other movement. \
Subscriptions: Receives pose from `/amcl_pose`, goals from `/goal_position`, `/elevator/done` to know when to stop, and `/scan` for LiDAR scans. \
Publishers: `/cmd_vel_acc` for velocity commands, `goal_position_cb` to publish when a goal has been reached.

`SDH7_enter_elevator.py`: Contains general logic of entering the 7th floor elevator. \
Subscriptions: `/goal_position_cb` to know when a goal has been reached. \
Publishers: `/goal_position` to publish new goals from a list for `position_controller.py` to follow. `/elevator/done` to publish when done so `run_integrated.py` can kill the launch file that spawned this node. 

TODO: `SDH7_exit_elevator.py`, `SDH2_enter_elevator.py`, `SDH2_exit_elevator.py`

## Elevater Utils

Requirements:

-cv2, cv_bridge

### Elevator Door Detection

TODO: Currently only supports 7th floor.

We use a LiDAR to determine whether or not an elevator door has opened in front of us. This is done by standing in front of the right side elevator and checking if there are enough LiDAR scans from within the elevator for us to believe the door is open. If it is not open for a while after hearing the elevator "ding", we conclude that the left elevator door must have opened.

Outputs:
Topic: `/elevator/detection`
Message type: Int32
Information: 7 if 7th floor elevator detected. 2 if 2nd floor elevator detected. Else, does not publish.

To run:

```bash
rosrun elevator_utils lidar_elevator_detection.py
```

NOTE: The rest of "Elevator Door Detection" is deprecated, but is kept since the script is still in the repository.
This uses optic flow for detecting which elevator door opens. Robot must stay still to use, with 2 elevator doors in the frame.

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
