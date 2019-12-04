#! /usr/bin/env python

import rospy
import numpy as np 

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan

from tf.transformations import euler_from_quaternion

class Position_Controller:
	def __init__(self):
		print('Initializing position controller')

		# Collision avoidance variables
		self.shape = np.array([.45, .35]) # half of the x, y dimensions
		self.refresh_rate = .2 # pessimistic assumption: 5hz updates for avoidance
		
		# First set the inital variables
		self.received_goal = False
		self.received_odom = False
		self.state = Twist()
		self.prev_state = None
		self.prev_cmd = Twist()
		self.goal_position = Twist()
		self.last_scan = None

		# Set the publishers and subscribers
		self.publish_command = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
		self.publish_goal_reached = rospy.Publisher('/goal_position_cb', Bool, queue_size=1)
		rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.update_state)
		rospy.Subscriber('/goal_position', Twist, self.update_goal_postion)
		rospy.Subscriber('/elevator/done', Bool, self.pub_stop)
		rospy.Subscriber('/scan', LaserScan, self.laser_cb)
		rospy.Timer(rospy.Duration(1/30.0), self.controller_cb)
		
		# Stop command
		self.stop_cmd = Twist()
		self.stop_cmd.linear.x = 0
		self.stop_cmd.linear.y = 0
		self.stop_cmd.angular.z = 0
		
		# PD parameters
		self.Kp_x = 1.
		self.Kd_x = .8
		self.Kp_y = 1.
		self.Kd_y = .8
		self.Kp_yaw = .5
		self.Kd_yaw = .2

		# Limit output commands
		self.max_xvel = .9 # Meters/sec
		self.max_yvel = .9 # Meters/sec 
		self.max_yaw  = 1.5 # Radians/sec

		# Limit acceleration to mitigate sensor drift
		self.max_lin_accel = .5 # Meters / (sec*iteration) (Techinically not m/s^2)
		self.max_ang_accel = .5
 
# =========== ROS Update Functions =========================================
	# Publish stop when done with elevators
	def pub_stop(self, msg):
		if msg.data:
			self.publish_command.publish(self.stop_cmd)

	# When receiving a new goal from enter/exit elevator, update goal.
	def update_goal_postion(self, wp):
		self.goal_position = wp 
		self.received_goal = True

	# Method for converting state quaternions to twist eulers
	def quat_to_eul(self, orientation):
		return euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))[2]

	# Update state based on odom and store it in a 
	def update_state(self, state_odom):
		# Make sure there is an inital state first. 
		self.state.linear.x = state_odom.pose.pose.position.x
		self.state.linear.y = state_odom.pose.pose.position.y
		self.state.angular.z = self.quat_to_eul(state_odom.pose.pose.orientation)
		
		self.received_odom = True

	# Update latest scan.
	def laser_cb(self, scan):
		self.last_scan = scan

	# Based on timer callbacks, calculate new commands based on current state and goal.
	def controller_cb(self, event):
		# Do nothing if haven't received odom or goal yet.
		if not self.received_odom or not self.received_goal:
			self.publish_command.publish(self.stop_cmd)
			self.prev_cmd = self.stop_cmd
			return

		# Check distance to goal
		x, y, yaw = self.state.linear.x, self.state.linear.y, self.state.angular.z
		goal_x, goal_y, goal_yaw = self.goal_position.linear.x, self.goal_position.linear.y, self.goal_position.angular.z
		errorx, errory, erroryaw = abs(x - goal_x), abs(y - goal_y), abs(yaw - goal_yaw)
		erroryaw = min(erroryaw, abs(2*np.pi - erroryaw))
		if errorx < .15 and errory < .15 and erroryaw < .3 and self.received_goal:
			self.publish_goal_reached.publish(Bool(data=True))
			self.received_goal = False
			return

		# Get new commands
		cmd_x, cmd_y = self.run_position_controller(self.goal_position)
		cmd_yaw = self.run_yaw_controller(self.goal_position.angular.z)

		# Limit acceleration of inputs		
		cmd_x = self.limit_accel(cmd_x, self.prev_cmd.linear.x, self.max_lin_accel)
		cmd_y = self.limit_accel(cmd_y, self.prev_cmd.linear.y, self.max_lin_accel)
		cmd_yaw = self.limit_accel(cmd_yaw, self.prev_cmd.angular.z, self.max_ang_accel)

		# Put outputs into a Twist
		cmd_out = Twist()
		cmd_out.linear.x = cmd_x
		cmd_out.linear.y = cmd_y
		cmd_out.angular.z = cmd_yaw

		# Publish output if no collisions found. Else stop.
		if not self.last_scan or self.check_collision(cmd_out):
			# cmd_out.linear.x = 0
			# cmd_out.linear.y = 0
			if(abs(cmd_out.linear.x) > .1):
				rel = .1/abs(cmd_out.linear.x)
				cmd_out.linear.x *= rel
			if(abs(cmd_out.linear.y) > .1):
				rel = .1/abs(cmd_out.linear.y)
				cmd_out.linear.y *= rel
		if self.last_scan:
		# if True:
			self.publish_command.publish(cmd_out)
		else:
			self.publish_command.publish(self.stop_cmd)

		# Update state
		self.prev_state = self.state

	# Limit the acceleration of the input command to prevent jerky movement
	# NOTE: Does not limit deceleration (don't want to override stop commands)
	def limit_accel(self, cmd, prev_cmd, max_accel):
		if cmd == 0:
			return 0
		if np.sign(cmd) == np.sign(prev_cmd):
			if abs(cmd) <= abs(prev_cmd):
				return cmd
			elif abs(cmd) - abs(prev_cmd) > max_accel:
				return np.sign(cmd) * (abs(prev_cmd) + max_accel)
		return cmd

	# Check collisions based on velocity command and latest scan.
	def check_collision(self, cmd):
		next_state = np.array([self.state.linear.x + cmd.linear.x * self.refresh_rate, \
					 self.state.linear.y + cmd.linear.y * self.refresh_rate])
		upper = next_state + self.shape
		lower = next_state - self.shape

		scan = self.last_scan
		angles = self.state.angular.z + np.pi + np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)
		ranges = np.array([scan.ranges]).T[:len(angles)]
		points = np.array([self.state.linear.x, self.state.linear.y]) + ranges * np.hstack((np.array([np.cos(angles)]).T, np.array([np.sin(angles)]).T))
		inside = np.logical_and((points <= upper), (points >= lower))
		votes = np.sum(2 == np.sum(inside, axis=1))
		return votes != 0

# ========== Main Position Controller ======================================

	# Based on desired goal point, calculate the x, y velocity commands
	def run_position_controller(self, goal):

		curr_pos = self.state
		x, y, yaw = curr_pos.linear.x, curr_pos.linear.y, curr_pos.angular.z
		goal_x, goal_y, goal_yaw = goal.linear.x, goal.linear.y, goal.angular.z

		# Calculate derivative terms
		if self.prev_state:
			dx, dy = x - self.prev_state.linear.x, y - self.prev_state.linear.y
			dx, dy = dx * np.cos(yaw) + dy * np.sin(yaw), dx * np.sin(yaw) + dy * np.cos(yaw)
		else:
			dx, dy = 0, 0

		# Calculate transformation and goal matrices
		tf_mat = np.array([[np.cos(yaw),-np.sin(yaw),x], \
			               [np.sin(yaw),np.cos(yaw),y],  \
						   [0,0,1.0]])
		goal = np.array([[np.cos(goal_yaw),-np.sin(goal_yaw),goal_x], \
			             [np.sin(goal_yaw),np.cos(goal_yaw),goal_y],  \
						 [0,0,1.0]])
		
		# Calculate state error (Rotate the goal into body frame)
		error = np.dot(np.linalg.inv(tf_mat), goal)
		errorx, errory = error[:2,2]

		# Use PD to calculate commands
		cmd_x = self.Kp_x*errorx + -self.Kd_x*dx
		cmd_y = self.Kp_y*errory + -self.Kd_y*dy
			  
		# Saturate commands to safe limits
		if(abs(cmd_x) > self.max_xvel):
			rel = self.max_xvel/abs(cmd_x)
			cmd_x *= rel
		if(abs(cmd_y) > self.max_yvel):
			rel = self.max_yvel/abs(cmd_y)
			cmd_y *= rel

		return cmd_x, cmd_y

	# Based on desired yaw, calculate the yaw input needed.
	def run_yaw_controller(self, goal_yaw):
		yaw = self.state.angular.z 
		
		if self.prev_state:
			dyaw = yaw - self.prev_state.angular.z
		else:
			dyaw = .0

		erroryaw = (goal_yaw - yaw)

		# Keep output between -PI and PI
		erroryaw %= 2*np.pi
		if erroryaw > np.pi:
			erroryaw -= 2*np.pi

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
