#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from set_goal import Navigation

class WaypointPublisher():
    def __init__(self, path):
        self.wp_pub = rospy.Publisher('/goal_position', Twist, queue_size=1, latch=True)
        self.done_pub = rospy.Publisher('/elevator/done', Bool, queue_size=1)
        rospy.Subscriber("/goal_position_cb", Bool, self.wp_publisher_cb)
        self.path = path
        self.wp_publisher()

    def wp_publisher(self):
        if len(self.path) > 0:
            wp = self.path.pop(0)
            print("Going to {} {} {}".format(wp.linear.x, wp.linear.y, wp.angular.z))
            self.wp_pub.publish(wp)
        else:
            done = Bool()
            done.data = True
            print("Done with entering SDH7 elevator!")
            self.done_pub.publish(done)

    def wp_publisher_cb(self, msg):
        if msg.data:
            self.wp_publisher()

if __name__ == '__main__':
    rospy.init_node('waypoint_publisher')

    # Teleportation if using sim
    # if rospy.get_param('use_sim_time') == True:
    move = Navigation()
    
        # r = rospy.Rate(1)
        # r.sleep()
    path = []
    side = rospy.get_param('/elevator')
    if(side == "left"):
        outside = Twist()
        outside.linear.x, outside.linear.y, outside.angular.z = 51.8222732544, 7.04585456848, 3.14159
    if(side == "right"):
        outside = Twist()
        outside.linear.x, outside.linear.y, outside.angular.z = 51.8222732544, 3.97035646439, 3.14159
    #TODO: wait for callback of elevator open door
    wp_pub = WaypointPublisher([outside])
    rospy.spin()
