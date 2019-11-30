#!/usr/bin/env python
import rospy
import roslaunch
import threading

from std_msgs.msg import Bool
from set_goal import Navigation

if __name__ == '__main__':

	def elevator_done_cb(msg):
		global done
		if launcher:
			done = True
			print(launcher)

	def launch(file):
		global launcher
		launcher = roslaunch.parent.ROSLaunchParent(uuid, launch_files[file])
		launcher.start()

	rospy.init_node('SDH_controller', anonymous=True)
	elevator_sub = rospy.Subscriber('/elevator/done', Bool, elevator_done_cb)
	launcher = None
	done = False
	
	SDH7_launch_file = roslaunch.rlutil.resolve_launch_arguments(['navigation', 'robot_nav_sim_7.launch'])
	SDH7_enter_elevator_file = roslaunch.rlutil.resolve_launch_arguments(['navigation', 'SDH7_enter_elevator_sim.launch'])
	SDH2_exit_elevator_file = roslaunch.rlutil.resolve_launch_arguments(['navigation', 'SDH2_exit_elevator_sim.launch'])
	SDH2_launch_file = roslaunch.rlutil.resolve_launch_arguments(['navigation', 'robot_nav_sim_2.launch'])
	# SDH2_elevator_file = roslaunch.rlutil.resolve_launch_arguments(['navigation', 'SDH2_elevator.launch'])

	launch_files = {}
	launch_files['SDH7'] = SDH7_launch_file
	launch_files['SDH7_enter_elevator'] = SDH7_enter_elevator_file
	launch_files['SDH2_exit_elevator'] = SDH2_exit_elevator_file
	launch_files['SDH2'] = SDH2_launch_file
	# launch_files['SDH2_elevator_file'] = SDH2_elevator_file

	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
	roslaunch.configure_logging(uuid)

	# SDH7 start navigation
	# launch('SDH7')
	# print('Starting sim move_base')
	# while not rospy.is_shutdown():
	# 	r = rospy.Rate(1) # 10hz
	# 	r.sleep()
	# 	with launcher_lock:
	# 		if launcher is None:
	# 			break

	# Start SDH 7 Elevator script
	launch('SDH7_enter_elevator')
	print('Transferring control to position controller')
	print('Entering SDH7 elevator')
	while not rospy.is_shutdown():
		r = rospy.Rate(1) # 1hz
		r.sleep()
		if done is True:
			launcher.shutdown()
			launcher = None
			break
				
	print('Congrats on making it to the SDH 7 elevator!')

	print('Teleporting to floor 2 elevator')

	launch('SDH2_exit_elevator')
	print('Transferring control to position controller')
	print('Exiting SDH2 elevator')
	while not rospy.is_shutdown():
		r = rospy.Rate(1) # 10hz
		r.sleep()
		if launcher is None:
			break
	print('Congrats on making it to SDH2!')

	print('Starting floor 2 move_base')
	launch('SDH2')
	while not rospy.is_shutdown():
		r = rospy.Rate(1) # 10hz
		r.sleep()
		if launcher is None:
			break
	print('Reached cafe doors!')