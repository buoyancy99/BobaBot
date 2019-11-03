#!/usr/bin/env python
from __future__ import division
import rospy
from sensor_msgs.msg import Image
from roborts_msgs.msg import GimbalAngle
from geometry_msgs.msg import Twist, Vector3

import cv2
from cv_bridge import CvBridge, CvBridgeError

mouse_x = 0
mouse_y = 0
mouse_left = False

ANGULAR_VEL = 0.5
LINEAR_VEL = 1.2

def run_teleop():
    rospy.init_node('teleop_controller')
    bridge = CvBridge()
    cv2.namedWindow("TeleOp", 0)
    gimbal_pub = rospy.Publisher('/cmd_gimbal_angle', GimbalAngle, queue_size=10)
    chassis_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def mouse_callback(event, x, y, flags, param):
        global mouse_x
        global mouse_y
        global mouse_left
        
        mouse_x = (x - 640) / 1280
        mouse_y = (y - 360) / 720

        if event == cv2.EVENT_LBUTTONDOWN:
            mouse_left = True

        if event == cv2.EVENT_LBUTTONUP:
            mouse_left = False
        
    def image_callback(img_msg):
        try:
            cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        cv2.imshow("TeleOp", cv_image)
        k = cv2.waitKey(3)

        scaling_yaw = -2.0
        scaling_pitch = 0.8
        gimbal_cmd = GimbalAngle()
        gimbal_cmd.yaw_mode = False
        gimbal_cmd.pitch_mode = False
        gimbal_cmd.yaw_angle = mouse_x * scaling_yaw
        gimbal_cmd.pitch_angle = mouse_y * scaling_pitch

        if k == ord('s'):
            chassis_cmd = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, ANGULAR_VEL))
        elif k == ord('d'):
            chassis_cmd = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, -ANGULAR_VEL))
        elif mouse_left:
            chassis_cmd = Twist(Vector3(- LINEAR_VEL * mouse_y, 0.0, 0.0), Vector3(0.0, 0.0, - ANGULAR_VEL * mouse_x * 3.0))
        else:
            chassis_cmd = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))

        gimbal_pub.publish(gimbal_cmd)
        chassis_pub.publish(chassis_cmd)

    cv2.setMouseCallback('TeleOp', mouse_callback)
    sub_image = rospy.Subscriber("/mynteye/left/image_color", Image, image_callback)

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == "__main__":
    run_teleop()
    