#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionResult


class Navigation:

    def __init__(self):
        self.ps_cov_pub = rospy.Publisher(
            'initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.nav_result_sub = rospy.Subscriber(
            'move_base/result', MoveBaseActionResult, self.getNavResult, queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.reachGoal = False

    def getNavResult(self, data):
        '''
        http://docs.ros.org/api/actionlib_msgs/html/msg/GoalStatus.html
        uint8 PENDING=0
        uint8 ACTIVE=1
        uint8 PREEMPTED=2
        uint8 SUCCEEDED=3
        uint8 ABORTED=4
        uint8 REJECTED=5
        uint8 PREEMPTING=6
        uint8 RECALLING=7
        uint8 RECALLED=8
        uint8 LOST=9
        '''
        self.reachGoal = data.status.status == 3

    def set_init_pose(self,
                      pos=[49.4471969604, -0.919650554657, 0],
                      ori=[0, 0, -0.13963701777, 0.990202758665],
                      cov=[0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787],
                      floor=2,
                      frame_id="map"):
        ps = PoseStamped()
        ps_cov = PoseWithCovarianceStamped()
        ps.pose.position.x = pos[0]
        ps.pose.position.y = pos[1]
        ps.pose.position.z = pos[2]
        ps.pose.orientation.x = ori[0]
        ps.pose.orientation.y = ori[1]
        ps.pose.orientation.z = ori[2]
        ps.pose.orientation.w = ori[3]
        ps.header.frame_id = frame_id
        ps.header.stamp = rospy.get_rostime()
        ps_cov.header = ps.header
        ps_cov.pose.pose = ps.pose
        ps_cov.pose.covariance = cov

        self.ps_cov_pub.publish(ps_cov)

    def movebase_client(self, pos=[71.796569824, -6.63157987595, 0], 
                        ori=[0, 0, -0.0293482956565, 0.999569245997], 
                        floor=2, frame_id="map"):

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        # set goal
        goal.target_pose.pose.position.x = pos[0]
        goal.target_pose.pose.position.y = pos[1]
        goal.target_pose.pose.position.z = pos[2]
        goal.target_pose.pose.orientation.x = ori[0]
        goal.target_pose.pose.orientation.y = ori[1]
        goal.target_pose.pose.orientation.z = ori[2]
        goal.target_pose.pose.orientation.w = ori[3]

        # Sends the goal to the action server.
        self.client.send_goal(goal)
        # Waits for the server to finish performing the action.
        wait = self.client.wait_for_result()
        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            # Result of executing the action
            return self.client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('set_goal')
        move = Navigation()
        move.set_init_pose()
        result = move.movebase_client()
       # move.set_init_pose([-16.692276001,1.4976606369,0],[0,0,-0.694762724396,0.719239012283],[0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787])
        # rospy.sleep(10)
        # result = move.movebase_client()
        if move.reachGoal:
            rospy.loginfo("Reached position")
        # r = rospy.Rate(3) # 10hz
        # while result != actionlib.SimpleGoalState.DONE:
        #     result = move.movebase_client()
        #     r.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
