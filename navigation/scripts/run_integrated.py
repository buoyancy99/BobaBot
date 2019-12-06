#!/usr/bin/env python
import rospy
import roslaunch
import threading

from std_msgs.msg import Bool
from set_goal import Navigation

if __name__ == '__main__':

	def elevator_done_cb(msg):
		global launcher
		if launcher:
			launcher.shutdown()
			launcher = None

	def launch(file):
		global launcher
		launcher = roslaunch.parent.ROSLaunchParent(uuid, launch_files[file])
		launcher.start()

	def wait_finish():
		global launcher
		while not rospy.is_shutdown():
			r = rospy.Rate(1)
			try:
				r.sleep()
			except:
				pass
			if launcher is None:
				break

	rospy.init_node('SDH_controller', anonymous=True)
	elevator_sub = rospy.Subscriber('/elevator/done', Bool, elevator_done_cb)

	launcher = None
	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
	roslaunch.configure_logging(uuid)

	SDH7_enter_elevator_file = roslaunch.rlutil.resolve_launch_arguments(['navigation', 'SDH7_enter_elevator.launch'])
	SDH2_exit_elevator_file = roslaunch.rlutil.resolve_launch_arguments(['navigation', 'SDH2_exit_elevator.launch'])
	SDH2_enter_elevator_file = roslaunch.rlutil.resolve_launch_arguments(['navigation', 'SDH2_enter_elevator.launch'])
	SDH7_exit_elevator_file = roslaunch.rlutil.resolve_launch_arguments(['navigation', 'SDH7_exit_elevator.launch'])

	launch_files = {}
	launch_files['SDH7_enter_elevator'] = SDH7_enter_elevator_file
	launch_files['SDH2_exit_elevator'] = SDH2_exit_elevator_file
	launch_files['SDH2_enter_elevator'] = SDH2_enter_elevator_file
	launch_files['SDH7_exit_elevator'] = SDH7_exit_elevator_file

	move = Navigation()
	raw_input("Tell me when to start")
	# SDH7 start navigation
	move.set_init_pose(pos=[25.4214736938,4.34529781342,0],ori=[0,0,0.0795551282507,0.996830467817],cov=[0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787])
	result = move.movebase_client(pos=[50.009576416,4.13174282913,0],ori=[0,0,0.0123519308175,0.999923711993])

	# # Uncomment below for unit testing 7th floor elevator.
	# move.set_init_pose(pos=[50.009576416,4.13174282913,0],ori=[0,0,0.0123519308175,0.999923711993],cov=[0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787])

	# Start SDH 7 Elevator script
	print('Starting SDH7 elevator script')
	launch('SDH7_enter_elevator')
	wait_finish()
	print('Congrats on making it to the SDH 7 elevator!')

	# Start 7 to 2 to exit script
	print('Starting inside elevator 7 to 2 script')
	launch('SDH2_exit_elevator')
	wait_finish()
	print('Congrats on making it to SDH 2!')
	# move.set_init_pose(pos=[-24.5,-0.4,0],ori=[0,0,0.999993843881,0.00350887452617],cov=[0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787])

	# Go to cafe
	# result = move.movebase_client(pos=[-49.2158203125,-7.58403015137,0],ori=[0,0,0.708139986808,0.706072063661])
	result = move.movebase_client(pos=[-42.6,2.064,0],ori=[0,0,-0.6983,0.7158])
	raw_input("Tell me when to return")
	
	# Go back to elevator
	result = move.movebase_client(pos=[-25.4651012421,-0.54517769814,0],ori=[0,0,0.0123519308175,0.999923711993])
	result = move.movebase_client(pos=[-25.4651012421,-3.54517769814,0],ori=[0,0,0.0123519308175,0.999923711993])
	# move.set_init_pose(pos=[-25.5,-3.5,0],ori=[0,0,0.0123519308175,0.999923711993],cov=[0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787])
        
	# Start SDH 7 Elevator script
	print('Starting SDH2 elevator script')
	launch('SDH2_enter_elevator')
	wait_finish()
	print('Congrats on making it to the SDH 2 elevator!')

	# Start 7 to 2 to exit script
	print('Starting inside elevator 2 to 7 script')
	launch('SDH7_exit_elevator')
	wait_finish()
	print('Congrats on making it to SDH 7!')

	# Return to meeting room
	result = move.movebase_client(pos=[24.7214736938,6.34529781342,0],ori=[0,0,0.0795551282507,0.996830467817])

	print('Done!')

	rospy.spin()