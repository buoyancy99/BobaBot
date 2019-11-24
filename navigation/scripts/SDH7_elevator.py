#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class WaypointPublisher():
    def __init__(self, path):
        self.wp_pub = rospy.Publisher('/goal_position', Twist, queue_size=1)
        self.done_pub = rospy.Publisher('/elevator_done', Bool, queue_size=1)
        rospy.Subscriber("/goal_position_cb", Bool, self.wp_publisher_cb)
        self.path = path
        self.wp_publisher()

    def wp_publisher(self):
        if len(self.path) > 0:
            wp = self.path.pop(0)
            print("Going to", wp.linear.x, wp.linear.y, wp.angular.z)
            self.wp_pub.publish(wp)
        else:
            done = Bool()
            done.data = True
            self.done_pub.publish(done)

    def wp_publisher_cb(self, msg):
        if msg.data:
            self.wp_publisher()

if __name__ == '__main__':
    rospy.init_node('waypoint_publisher')
    path = []
    outside = Twist()
    outside.linear.x, outside.linear.y = 49.15, 7.2
    inside = Twist()
    inside.linear.x, inside.linear.y, inside.angular.z = 53.2, 7.0, 3.14
    wp_pub = WaypointPublisher([outside, inside])
    rospy.spin()
