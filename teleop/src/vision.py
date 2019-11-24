#!/usr/bin/env python
from __future__ import division
import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

def run_teleop():
    rospy.init_node('testnode')
    bridge = CvBridge()
    cv2.namedWindow("image", 0)

    cv2.createTrackbar('low_H', 'image', 0,179, lambda x:None)
    cv2.createTrackbar('high_H', 'image', 0,179, lambda x:None)
    cv2.createTrackbar('low_S', 'image', 0,255, lambda x:None)
    cv2.createTrackbar('high_S', 'image', 0,255, lambda x:None)
    cv2.createTrackbar('low_V', 'image', 0,255, lambda x:None)
    cv2.createTrackbar('high_V', 'image', 0,255, lambda x:None)

    mask_buffer = np.zeros((6, 240, 640))

    def image_callback(img_msg, args):
        try:
            frame = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        frame = frame[:240]
        frame = cv2.blur(frame, (15, 15))
        # frame[:, 200:-200]=0

        ilowH = cv2.getTrackbarPos('low_H', 'image')
        ihighH = cv2.getTrackbarPos('high_H', 'image')
        ilowS = cv2.getTrackbarPos('low_S', 'image')
        ihighS = cv2.getTrackbarPos('high_S', 'image')
        ilowV = cv2.getTrackbarPos('low_V', 'image')
        ihighV = cv2.getTrackbarPos('high_V', 'image')

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_hsv = np.array([ilowH, ilowS, ilowV])
        higher_hsv = np.array([ihighH, ihighS, ihighV])
        mask = cv2.inRange(hsv, lower_hsv, higher_hsv)

        mask_buffer = args[0]
        mask_buffer[:-1] = mask_buffer[1:]
        mask_buffer[-1] = mask > 127
        mask = (np.prod(mask_buffer, axis=0) * 255.0).astype(np.uint8)
        frame = cv2.bitwise_and(frame, frame, mask=mask)

        cv2.imshow("image", frame)
        k = cv2.waitKey(5)

    sub_image = rospy.Subscriber("/mynteye/left/image_color", Image, image_callback, (mask_buffer, ))

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == "__main__":
    run_teleop()
    
