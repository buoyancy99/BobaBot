#Voice Recognition

Packages Needed: In requirements.txt inside voice_utils

###Elevator Speech
Use to call human to press button:
Use rosrun sound_play Trial.py
If human found inside elevator:
Use rosrun sound_play Trial2.py

###Speech Recognition Part

Voice recognition stuff is in recognizer.py which is written as a ROS service such that it can read input from mic and publish back to the node.

rosrun sound_play recog.py
