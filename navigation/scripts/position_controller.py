#! /usr/bin/env python

import rospy
import numpy as np 

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from roborts_msgs.msg import TwistAccel
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan

from tf.transformations import euler_from_quaternion

class Position_Controller:
	def __init__(self):
		print('Initializing position controller')
		self.shape = np.array([.45, .45]) # half of the x, y dimensions
		self.refresh_rate = .2 # assume 10hz updates. 
		# First set the inital variables
		self.received_goal = False
		self.received_odom = False
		self.state = Twist()
		self.last_state = Twist()
		self.goal_position = Twist()
		self.last_scan = None

		# Set the publishers and subscribers
		self.publish_command = rospy.Publisher('/cmd_vel_acc',TwistAccel,queue_size=1)
		self.publish_goal_reached = rospy.Publisher('/goal_position_cb', Bool, queue_size=1)
		rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.update_state)
		rospy.Subscriber('/goal_position', Twist, self.update_goal_postion)
		rospy.Subscriber('/elevator_done', Bool, self.pub_stop)
		rospy.Subscriber('/scan', LaserScan, self.laser_cb)
		rospy.Timer(rospy.Duration(1/30.0), self.controller_cb)
		
		stop = TwistAccel()
		stop.twist.linear.x = 0
		stop.twist.linear.y = 0
		stop.twist.angular.z = 0
		
		self.stop_cmd = stop

		self.Kp_x = 1.
		self.Kd_x = .8
		self.Kp_y = 1.
		self.Kd_y = .8
		self.Kp_yaw = .5
		self.Kd_yaw = .2

		self.prev_state = None
		self.prev_cmd = TwistAccel()

		self.max_xvel = .9 # Meters /sec
		self.max_yvel = .9 # Meters /sec 
		self.max_yaw  = 1.5 # radians sec

		# Limit acceleration to mitigate sensor drift
		self.max_lin_accel = .5 # Meters / (sec*iteration) (Techinically not m/s^2)
		self.max_ang_accel = .5
 
# =========== ROS Update Functions =========================================
	def pub_stop(self, msg):
		if msg.data:
			self.publish_command.publish(self.stop_cmd)

	def update_goal_postion(self, wp):
		self.goal_position = wp 
		self.received_goal = True

	def quat_to_eul(self, orientation):
		return euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))[2]

	def update_state(self, state_odom):
		# Make sure there is an inital state first. 
		self.state.linear.x = state_odom.pose.pose.position.x
		self.state.linear.y = state_odom.pose.pose.position.y
		self.state.angular.z = self.quat_to_eul(state_odom.pose.pose.orientation)
		
		self.received_odom = True

	def laser_cb(self, scan):
		self.last_scan = scan

	def controller_cb(self, event):
		if not self.received_odom or not self.received_goal:
			self.publish_command.publish(self.stop_cmd)
			self.prev_cmd = self.stop_cmd
			return

		# print('Update State', self.state.linear.x, self.state.linear.y, self.state.angular.z)		

		cmd_x, cmd_y = self.run_position_controller()
		cmd_yaw = self.run_yaw_controller()
		
		x, y, yaw = self.state.linear.x, self.state.linear.y, self.state.angular.z
		goal_x, goal_y, goal_yaw = self.goal_position.linear.x, self.goal_position.linear.y, self.goal_position.angular.z
		errorx, errory, erroryaw = abs(x - goal_x), abs(y - goal_y), abs(yaw - goal_yaw)
		erroryaw = min(erroryaw, abs(2*np.pi - erroryaw))
		# print(errorx, errory, erroryaw)
		if errorx < .15 and errory < .15 and erroryaw < .3 and self.received_goal:
			self.publish_goal_reached.publish(Bool(data=True))
			self.received_goal = False

		# print(cmd_x, cmd_y, cmd_yaw)
		
		cmd_x = self.limit_accel(cmd_x, self.prev_cmd.twist.linear.x, self.max_lin_accel)
		cmd_y = self.limit_accel(cmd_y, self.prev_cmd.twist.linear.y, self.max_lin_accel)
		cmd_yaw = self.limit_accel(cmd_yaw, self.prev_cmd.twist.angular.z, self.max_ang_accel)

		cmd_out = TwistAccel()
		cmd_out.twist.linear.x = cmd_x
		cmd_out.twist.linear.y = cmd_y
		cmd_out.twist.angular.z = cmd_yaw
		# print("Command", cmd_x, cmd_y, cmd_yaw)
		self.prev_state = self.state
		if self.last_scan and not self.check_collision(cmd_out.twist):
		# if True:
			self.publish_command.publish(cmd_out)
		else:
			self.publish_command.publish(self.stop_cmd)

	def limit_accel(self, cmd, prev_cmd, max_accel):
		if cmd == 0:
			return 0
		if np.sign(cmd) == np.sign(prev_cmd):
			if abs(cmd) <= abs(prev_cmd):
				return cmd
			elif abs(cmd) - abs(prev_cmd) > max_accel:
				return np.sign(cmd) * (abs(prev_cmd) + max_accel)
		return cmd

	def check_collision(self, cmd):
		next_state = np.array([self.state.linear.x + cmd.linear.x * self.refresh_rate, \
					 self.state.linear.y + cmd.linear.y * self.refresh_rate])
		upper = next_state + self.shape
		lower = next_state - self.shape

		scan = self.last_scan
		angles = self.state.angular.z + np.pi + np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)
		ranges = np.array([scan.ranges]).T[:len(angles)]
		points = np.array([self.state.linear.x, self.state.linear.y]) + ranges * np.hstack((np.array([np.cos(angles)]).T, np.array([np.sin(angles)]).T))
		# print(points)
		inside = np.logical_and((points <= upper), (points >= lower))
		# print(inside.shape)
		# print(inside)
		votes = np.sum(2 == np.sum(inside, axis=1))
		return votes != 0

	def check_reached(self):
		threshhold_xy = .01 #Meters
		threshhold_angular = .19634954084 #pi/16
		if (abs(self.goal.position.linear.x - self.state.linear.x) < threshhold_xy 
			and abs(self.goal.position.linear.y - self.state.linear.y) < threshhold_xy
			and abs(self.goal_position.angular.z - self.state.angular.z) < threshhold_angular):
			return True
		else:
			return False

# ========== Main Position Controller ======================================

	def run_position_controller(self):

		goal = self.goal_position #twist
		curr_pos = self.state #Twist as well
		x, y, yaw = curr_pos.linear.x, curr_pos.linear.y, curr_pos.angular.z
		if self.prev_state:
			dx, dy = x - self.prev_state.linear.x, y - self.prev_state.linear.y
			dx, dy = dx * np.cos(yaw) + dy * np.sin(yaw), dx * np.sin(yaw) + dy * np.cos(yaw)
		else:
			dx, dy = 0, 0
		goal_x, goal_y, goal_yaw = goal.linear.x, goal.linear.y, goal.angular.z

		tf_mat = np.array([[np.cos(yaw),-np.sin(yaw),x], \
			               [np.sin(yaw),np.cos(yaw),y],  \
						   [0,0,1.0]])
		goal = np.array([[np.cos(goal_yaw),-np.sin(goal_yaw),goal_x], \
			             [np.sin(goal_yaw),np.cos(goal_yaw),goal_y],  \
						 [0,0,1.0]])

		# print '=============='
		# print "goal",goal_x, goal_y, goal_yaw

		##############  INSERT CODE BELOW ######################
		
		# Calculate state error (HINT!! Rotate the errors into body frame)
		error = np.dot(np.linalg.inv(tf_mat), goal)

		errorx, errory = error[:2,2]
		
		# errorx,errory,erroryaw = goal.linear.x - curr_pos.linear.x , goal.linear.y - curr_pos.linear.y, goal.angular.z - curr_pos.angular.z
		# erroryaw = -erroryaw

		# print "State Error: x: %.2f y: %.2f" %(errorx,errory)

		# Calculate twist commands in each axis
		# state = self.state
		# calculate vel 
		# dx = self.state.linear.x - self.last_state.linear.x 
		# dy = self.state.linear.y - self.last_state.linear.y 
		# dyaw = self.state.angular.y - self.last_state.angular.y 

		#dx_global = dx*np.cos(yaw) + dy*np.sin(yaw)
		#dy_global = -dx*np.sin(yaw) + dy*np.cos(yaw)

		cmd_x = self.Kp_x*errorx + -self.Kd_x*dx
		cmd_y = self.Kp_y*errory + -self.Kd_y*dy
		
		#print "Unbounded Command: x: %.2f y: %.2f yaw: %.2f" %(cmd_x,cmd_y,cmd_yaw)
	  
		# Saturate commands to safe limits
		if(abs(cmd_x) > self.max_xvel):
			rel = self.max_xvel/abs(cmd_x)
			cmd_x *= rel
		if(abs(cmd_y) > self.max_yvel):
			rel = self.max_yvel/abs(cmd_y)
			cmd_y *= rel

		return cmd_x, cmd_y 
		#print "Saturated Command: x: %.2f y: %.2f yaw: %.2f" %(cmd_x,cmd_y,cmd_yaw)


	def run_yaw_controller(self):
		goal_yaw = self.goal_position.angular.z
		yaw = self.state.angular.z 
		
		if self.prev_state:
			dyaw = yaw - self.prev_state.angular.z
		else:
			dyaw = .0

		erroryaw = (goal_yaw - yaw)
		erroryaw %= 2*3.14159
		if erroryaw > 3.14159:
			erroryaw -= 2*3.14159
		# print "State Error: yaw: %.2f" %(erroryaw)

		cmd_yaw = self.Kp_yaw*erroryaw + -self.Kd_yaw*dyaw
		if(cmd_yaw > 0):
			cmd_yaw = min(cmd_yaw,self.max_yaw)
		else:
			cmd_yaw = max(cmd_yaw,-self.max_yaw)

		return cmd_yaw


if __name__ == '__main__':
	rospy.init_node('position_controller')
	controller = Position_Controller()
	print 'Controller Running'

rospy.spin()
