#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class Navigation:
    def __init__(self):
        self.ps_cov_pub = rospy.Publisher(
            'init_pose_conv', PoseWithCovarianceStamped, queue_size=1)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def set_init_pose(self, floor=2,
                        pos=[-0.050385594368, -1.39488351345],
                        cov=[
                            0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.06853892326654787
                        ], frame_id = "map"):
        ps = PoseStamped()
        ps_cov = PoseWithCovarianceStamped()
        ps.pose.position.x = pos[0]
        ps.pose.position.y = pos[1]
        ps.pose.position.z = 0
        ps.header.frame_id = frame_id
        ps.header.stamp = rospy.get_rostime()
        ps_cov.header = ps.header
        ps_cov.pose.pose = ps.pose
        ps_cov.pose.covariance = cov

        self.ps_cov_pub.publish(ps_cov)


    def movebase_client(self, floor=2, frame_id = "map"):
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        # set goal
        goal.target_pose.pose.position.x = -16.5200386047
        goal.target_pose.pose.position.y = 0.95269203186
        goal.target_pose.pose.orientation.z = -0.695976068805
        goal.target_pose.pose.orientation.w = 0.718064977318

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


# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('set_goal')
        move = Navigation()
        move.set_init_pose()
        rospy.sleep(3)
        result = move.movebase_client()
        # r = rospy.Rate(3) # 10hz
        # while result != actionlib.SimpleGoalState.DONE:
        #     result = move.movebase_client()
        #     r.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")