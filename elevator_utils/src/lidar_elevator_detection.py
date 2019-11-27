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
		self.vote_thresh = 20
		self.elevator_ul = np.array([55.5, 5.4])
		self.elevator_lr = np.array([52.8 , 2.55])

		self.publish_elevator = rospy.Publisher('/elevator/detection', Int32, queue_size=1)
		rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.update_state)
		rospy.Subscriber('/scan', LaserScan, self.laser_cb)

	def update_state(self, pose):
		self.state.linear.x = pose.pose.pose.position.x
		self.state.linear.y = pose.pose.pose.position.y
		self.state.angular.z = quat_to_eul(pose.pose.pose.orientation)

	def laser_cb(self, scan):
		angles = self.state.angular.z + np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)
		ranges = np.array([scan.ranges]).T
		points = np.array([self.state.linear.x, self.state.linear.y]) + ranges * np.hstack((np.array([np.cos(angles)]).T, np.array([np.sin(angles)]).T))
		# print(points)
		inside = np.logical_and((points <= self.elevator_ul), (points >= self.elevator_lr))
		# print(inside.shape)
		# print(inside)
		votes = np.sum(2 == np.sum(inside, axis=1))
		if votes >= self.vote_thresh:
			self.publish_elevator.publish(Int32(data=7))

if __name__ == '__main__':
	rospy.init_node('elevator_detector')
	detector = LidarElevatorDetector()

rospy.spin()
