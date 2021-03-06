#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32, Int8
from set_goal import Navigation

import time

class WaypointPublisher():
    def __init__(self, path):
        self.done = False
        self.wp_pub = rospy.Publisher('/goal_position', Twist, queue_size=1, latch=True)
        rospy.Subscriber("/goal_position_cb", Bool, self.wp_publisher_cb)
        self.path = path
        self.wp_publisher()

    def wp_publisher(self):
        if len(self.path) > 0:
            self.done = False
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
    elevator = None
    dinged = False
    wait_open = 3.5

    def elevator_cb(msg):
        if msg.data == 7:
            global elevator
            elevator = "right"
    def ding_detect_cb(msg):
        if msg.data != 0:
            global dinged
            dinged = True

    done_pub = rospy.Publisher('/elevator/done', Bool, queue_size=1)

    # Teleportation if using sim
    if rospy.get_param('use_sim_time') == True:
        move = Navigation()
        move.set_init_pose(pos=[50.009576416,4.13174282913,0],ori=[0,0,0.0123519308175,0.999923711993],cov=[0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787])
        r = rospy.Rate(1)
        r.sleep()

    # Wait for "ding"/user input before approaching elevator
    # raw_input("Tell me when to approach elevator")
    rospy.Subscriber("ding_value", Int8, ding_detect_cb)
    while not rospy.is_shutdown():
        r = rospy.Rate(10)
        try:
            r.sleep()
        except:
            pass
        if dinged:
            break

    outside_right = Twist()
    outside_right.linear.x, outside_right.linear.y, outside_right.angular.z = 51.4, 3.85, 0
    wp_pub = WaypointPublisher([outside_right])
    while not wp_pub.done and not rospy.is_shutdown():
        r = rospy.Rate(10)
        r.sleep()

    # # Test point to test position controller
    # test_pt = Twist()
    # test_pt.linear.x, test_pt.linear.y = 51.5, 6.8
    # path = [test_pt]
    # wp_pub = WaypointPublisher(path)
    # while not wp_pub.done and not rospy.is_shutdown():
    #     r = rospy.Rate(10)
    #     r.sleep()
    # # End test point code

    # Subscribe to elevator detection.
    rospy.Subscriber('/elevator/detection', Int32, elevator_cb)
    stop_wait = time.time() + wait_open
    path = []
    while not rospy.is_shutdown():
        r = rospy.Rate(1) # 3hz
        r.sleep()
        # If lidar detects right door, go in. Else after some time, go left.
        if elevator == "right":
            rospy.set_param('/elevator', 'right')
            inside1 = Twist()
            inside1.linear.x, inside1.linear.y = 53.4, 3.9
            inside2 = Twist()
            inside2.linear.x, inside2.linear.y, inside2.angular.z = 53.4, 3.9, 3.14
            path = [inside1, inside2]
            break
        elif time.time() >= stop_wait:
            # left if right didn't open
            rospy.set_param('/elevator', 'left')
            outside = Twist()
            outside.linear.x, outside.linear.y = 51.5, 6.8
            inside1 = Twist()
            inside1.linear.x, inside1.linear.y = 53.4, 6.8
            inside2 = Twist()
            inside2.linear.x, inside2.linear.y, inside2.angular.z = 53.4, 7.0, 3.14
            path = [outside, inside1, inside2]
            break
    wp_pub = WaypointPublisher(path)
    while not wp_pub.done and not rospy.is_shutdown():
        r = rospy.Rate(10)
        r.sleep()
    done_pub.publish(Bool(data=True))