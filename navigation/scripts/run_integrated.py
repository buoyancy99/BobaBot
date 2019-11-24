#!/usr/bin/env python
import rospy
import roslaunch

from std_msgs.msg import Bool
from set_goal import Navigation

if __name__ == '__main__':

	def elevator_done_cb(msg):
		if launcher:
			launcher.shutdown()

	def launch(file):
		launcher = roslaunch.parent.ROSLaunchParent(uuid, launch_files[file])
		launcher.start()

	rospy.init_node('SDH_controller', anonymous=True)
	elevator_sub = rospy.Subscriber('elevator_done', Bool, elevator_done_cb, queue_size=1)
	launcher = None

	SDH7_args = ['navigation', 'SDH7_elevator.launch']
	SDH7_launch_file = roslaunch.rlutil.resolve_launch_arguments(SDH7_args)

	# SDH2_args =
	# SDH2_launch_file = 

	launch_files = {}
	launch_files['SDH7'] = SDH7_launch_file
	# launch_files['SDH2'] = SDH2_launch_file

	move = Navigation()

	# SDH7 start navigation
	move.set_init_pose(pos=[24.7214736938,6.34529781342,0],ori=[0,0,0.0795551282507,0.996830467817],cov=[0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787])
	result = move.movebase_client(pos=[49.509576416,5.28544282913,0],ori=[0,0,0.0123519308175,0.999923711993])

	# Start SDH 7 Elevator script
	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
	roslaunch.configure_logging(uuid)
	launch('SDH7')
	rospy.spin()