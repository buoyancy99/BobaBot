#!/usr/bin/env python
import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32

from tf.transformations import euler_from_quaternion

import numpy as np

def quat_to_eul(orientation):
	return euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))[2]

class LidarElevatorDetector:
	def __init__(self):
		self.state = Twist()
		self.scan = LaserScan()
		self.vote_thresh = 4

		self.elevator_7_ul = np.array([[58, 5.4]])
		self.elevator_7_lr = np.array([[52.8 , 2.55]])

		self.elevator_2_ul = np.array([[-20.97, -2.06]])
		self.elevator_2_lr = np.array([[-23.47, -4.79]])

		self.publish_elevator = rospy.Publisher('/elevator/detection', Int32, queue_size=1)
		rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.update_state)
		rospy.Subscriber('/scan', LaserScan, self.laser_cb)

	def update_state(self, pose):
		self.state.linear.x = np.array(pose.pose.pose.position.x)
		self.state.linear.y = np.array(pose.pose.pose.position.y)
		self.state.angular.z = np.array(quat_to_eul(pose.pose.pose.orientation))

	def laser_cb(self, scan):
		# Make (N, ) array of angles
		angles = self.state.angular.z + np.pi + np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)
		# angles = self.state.angular.z + np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)
		
		# Make (N, 1) array of ranges
		ranges = np.array(scan.ranges)[:len(angles), None]
		# Make (N, 2) array of points
		points = np.array([self.state.linear.x, self.state.linear.y]) + ranges * np.hstack([np.cos(angles)[:, None], np.sin(angles)[:, None]])
		# Make boolean array of if scan points are within upper-left/lower-right of elevator bounding box.
		inside_7 = np.logical_and((points <= self.elevator_7_ul), (points >= self.elevator_7_lr))
		inside_2 = np.logical_and((points <= self.elevator_2_ul), (points >= self.elevator_2_lr))
		# Count number of points within upper-left/lower-right.
		votes_7 = np.sum(2 == np.sum(inside_7, axis=1))
		votes_2 = np.sum(2 == np.sum(inside_2, axis=1))

		# If more points within boundary than minimum needed, we have detected the 7th floor elevator.
		if votes_7 >= self.vote_thresh:
			self.publish_elevator.publish(Int32(data=7))
		if votes_2 >= self.vote_thresh:
			self.publish_elevator.publish(Int32(data=2))

if __name__ == '__main__':
	rospy.init_node('elevator_detector')
	detector = LidarElevatorDetector()

rospy.spin()
