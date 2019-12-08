#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_msgs.msg import Bool, Int8
from set_goal import Navigation

from time import sleep

class WaypointPublisher():
    def __init__(self, path):
        self.done = False
        self.wp_pub = rospy.Publisher('/goal_position', Twist, queue_size=1, latch=True)
        rospy.Subscriber("/goal_position_cb", Bool, self.wp_publisher_cb)
        self.path = path
        self.wp_publisher()

    def wp_publisher(self):
        if len(self.path) > 0:
            wp = self.path.pop(0)
            print("Going to {} {} {}".format(wp.linear.x, wp.linear.y, wp.angular.z))
            self.wp_pub.publish(wp)
        else:
            self.done = True

    def wp_publisher_cb(self, msg):
        if msg.data:
            self.wp_publisher()

if __name__ == '__main__':
    rospy.init_node('waypoint_publisher')
    done_pub = rospy.Publisher('/elevator/done', Bool, queue_size=1)
    floor_pub = rospy.Publisher('/floor_to_detect', Int8, queue_size=1)
    door_opened = False

    def floor_detect_cb(msg):
        global door_opened
        if msg.data == 1:
            door_opened = True
            floor_pub.publish(0)

    move = Navigation()
    pos = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped).pose.pose
    x, y = pos.position.x, pos.position.y
    qx, qy, qz, qw = pos.orientation.x, pos.orientation.y, pos.orientation.z, pos.orientation.w

    move.set_init_pose(pos=[x + 75.95,y + 7.55,0],ori=[qx,qy,qz,qw],cov=[0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787])

    sleep(1)

    side = rospy.get_param('/elevator')
    if(side == "left"):
        floor_pub.publish(3)
        outside = Twist()
        outside.linear.x, outside.linear.y, outside.angular.z = 50.5, 6.8, 3.14159
    if(side == "right"):
        floor_pub.publish(4)
        outside = Twist()
        outside.linear.x, outside.linear.y, outside.angular.z = 50.5, 3.85, 3.14159
    
    # Wait for callback of elevator open door
    # raw_input("tell me when to exit")
    rospy.Subscriber("/floor_value", Int8, floor_detect_cb)
    while not rospy.is_shutdown():
        r = rospy.Rate(10)
        try:
            r.sleep()
        except:
            pass
        if door_opened:
            break

    wp_pub = WaypointPublisher([outside])
    while not wp_pub.done and not rospy.is_shutdown():
        r = rospy.Rate(10)
        r.sleep()
    done_pub.publish(Bool(data=True))
