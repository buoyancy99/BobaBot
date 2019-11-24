#! /usr/bin/env python

import rospy

from roborts_msgs.msg import TwistAccel, GimbalAngle
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty, String
from keyboard.msg import Key

class Switch:
    def __init__(self):

        self.pub_cmd_vel = rospy.Publisher('cmd_vel_acc', TwistAccel, queue_size=1)
        self.cmd_vel_keyboard = TwistAccel();
        self.pub_gimbal_angle = rospy.Publisher('cmd_gimbal_angle', GimbalAngle, queue_size=1)
        self.cmd_gimbal_keyboard = GimbalAngle();
        self.cmd_gimbal_keyboard.yaw_mode = 1
        self.cmd_gimbal_keyboard.pitch_mode = 1
        self.cam_sel = rospy.Publisher('/camera_selection', String, queue_size=1)


        rospy.Subscriber('/keyboard/keydown', Key, self.process_keydown)
        rospy.Subscriber('/keyboard/keyup', Key, self.process_keyup)
        rospy.Subscriber('/joy', Joy, self.process_joy)

        self.robot_num = 0
        self.left_trigger = 0
        self.right_trigger = 0

    def process_joy(self, joy):
        cmd_vel = TwistAccel()
        cmd_gimbal = GimbalAngle()

        cmd_vel.twist.linear.x = joy.axes[1]*2
        cmd_vel.twist.linear.y = joy.axes[0]*2
        cmd_vel.twist.angular.z = joy.axes[3]

        cmd_gimbal.pitch_angle = joy.axes[4]
        cmd_gimbal.yaw_angle = (joy.axes[5] - joy.axes[2])

        if self.left_trigger == 0 and joy.buttons[4] == 1:
        	self.switch_robot(self.robot_num - 1)
        self.left_trigger = joy.buttons[4]

        if self.right_trigger == 0 and joy.buttons[5] == 1:
        	self.switch_robot(self.robot_num + 1)
        self.right_trigger = joy.buttons[5]

        self.pub_cmd_vel.publish(cmd_vel)
        self.pub_gimbal_angle.publish(cmd_gimbal)

    def process_keydown(self, key):
        if key.code == Key.KEY_UP:
            self.cmd_vel_keyboard.twist.linear.x = 1
        elif key.code == Key.KEY_DOWN:
            self.cmd_vel_keyboard.twist.linear.x = -1
        elif key.code == Key.KEY_RIGHT:
            self.cmd_vel_keyboard.twist.linear.y = -1
        elif key.code == Key.KEY_LEFT:
            self.cmd_vel_keyboard.twist.linear.y = 1
        elif key.code == Key.KEY_d:
            self.cmd_vel_keyboard.twist.angular.z = -1
        elif key.code == Key.KEY_a:
            self.cmd_vel_keyboard.twist.angular.z = 1
        elif key.code == Key.KEY_w:
            self.cmd_gimbal_keyboard.pitch_angle = 1
        elif key.code == Key.KEY_s:
            self.cmd_gimbal_keyboard.pitch_angle = -1
        elif key.code == Key.KEY_q:
            self.cmd_gimbal_keyboard.yaw_angle = -1
        elif key.code == Key.KEY_e:
            self.cmd_gimbal_keyboard.yaw_angle = 1
        elif key.code == Key.KEY_0:
            self.switch_robot(0)
        elif key.code == Key.KEY_1:
            self.switch_robot(1)
        elif key.code == Key.KEY_2:
            self.switch_robot(2)
        elif key.code == Key.KEY_3:
            self.switch_robot(3)
        elif key.code == Key.KEY_4:
            self.switch_robot(4)
        else:
            return
        self.pub_cmd_vel.publish(self.cmd_vel_keyboard)
        self.pub_gimbal_angle.publish(self.cmd_gimbal_keyboard)
       

    def process_keyup(self, key):
        if key.code == Key.KEY_w or key.code == Key.KEY_s:
            self.cmd_gimbal_keyboard.pitch_angle = 0
        elif key.code == Key.KEY_q or key.code == Key.KEY_e:
            self.cmd_gimbal_keyboard.yaw_angle = 0
        elif key.code == Key.KEY_d or key.code == Key.KEY_a:
            self.cmd_vel_keyboard.twist.angular.z = 0
        elif key.code == Key.KEY_UP or key.code == Key.KEY_DOWN:
            self.cmd_vel_keyboard.twist.linear.x = 0
        elif key.code == Key.KEY_RIGHT or key.code == Key.KEY_LEFT:
            self.cmd_vel_keyboard.twist.linear.y = 0
        else:
            return
        self.pub_cmd_vel.publish(self.cmd_vel_keyboard)
        self.pub_gimbal_angle.publish(self.cmd_gimbal_keyboard)

    def switch_robot(self, robot_num):
    	if robot_num < 0:
    		robot_num = 0
    	elif robot_num > 4:
    		robot_num = 4
    	self.robot_num = robot_num
        print('Switching to robot ' + str(robot_num) + '!')
        self.cmd_vel_keyboard = TwistAccel();
        self.cmd_gimbal_keyboard = GimbalAngle();
        self.pub_cmd_vel.publish(self.cmd_vel_keyboard)
        self.pub_gimbal_angle.publish(self.cmd_gimbal_keyboard)

        robot_name = 'roborts_' + str(robot_num) if robot_num != 0 else ""
        self.cam_sel.publish(String(robot_name))
        self.pub_cmd_vel = rospy.Publisher(robot_name + '/cmd_vel_acc', TwistAccel, queue_size=1)
        self.pub_gimbal_angle = rospy.Publisher(robot_name + '/cmd_gimbal_angle', GimbalAngle, queue_size=1)

if __name__ == '__main__':
    rospy.init_node('switch_node')
    switch = Switch()
    print('Switch Running')
    rospy.spin()
