#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int8

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import copy


#to_detect = 0

class FloorDetector:
    def __init__(self, args):
        self.args = args
        self.pub = rospy.Publisher('floor_value', Int8, queue_size=10)
        self.rate = rospy.Rate(5)
        self.to_detect = 0
        self.bridge = CvBridge()

        rospy.Subscriber("floor_to_detect", Int8, self.to_detect_callback)
        rospy.Timer(rospy.Duration(1/4.), self.color_image_callback)

    def fm(self, floor_sift, img2, sift, MIN_MATCH_COUNT=50):

        # find the keypoints and descriptors with SIFT
        kp1, des1 = floor_sift
        kp2, des2 = sift.detectAndCompute(img2,None)

        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)

        flann = cv2.FlannBasedMatcher(index_params, search_params)

        matches = flann.knnMatch(des1,des2,k=2)

        # store all the good matches as per Lowe's ratio test.
        good = []
        for m,n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)

        if len(good)>MIN_MATCH_COUNT:
            # src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            # dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

            # M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            # #matchesMask = mask.ravel().tolist()

            # h,w = img1[:,:,0].shape
            # pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            # dst = cv2.perspectiveTransform(pts,M)

            # img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)

            print('Floor!, ', len(good))
            return 1

        else:
            print("Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT))
            return 0


    def to_detect_callback(self, msg):
        self.to_detect = msg.data

    def color_image_callback(self, args):
        # print('callback')
        img_msg = rospy.wait_for_message("/mynteye/left/image_color", Image)
        #self.to_detect = 3
        # print('callback')
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))


        #cv2.imshow("door_detector", cv_image)
        
        if self.to_detect == 0:
            return
        # print('callback')
        floor_sift_list = self.args[0]
        floor_sift = floor_sift_list[self.to_detect-1]
        sift = self.args[1]
        threshold_list = self.args[2]
        floor_value = self.fm(floor_sift, cv_image, sift, MIN_MATCH_COUNT=threshold_list[self.to_detect-1])
        self.pub.publish(floor_value)
        # k = cv2.waitKey(3)

if __name__ == "__main__":
    rospy.init_node('floor_detector')

    floor2_left = cv2.imread('/home/rmcal/Projects/BobaBot/src/elevator_utils/src/sdh2_right_highdef.png')
    floor2_right = cv2.imread('/home/rmcal/Projects/BobaBot/src/elevator_utils/src/sdh2_right_highdef.png')
    floor7_left = cv2.imread('/home/rmcal/Projects/BobaBot/src/elevator_utils/src/floor7_left_fix.png')
    floor7_right = cv2.imread('/home/rmcal/Projects/BobaBot/src/elevator_utils/src/sdh7_right_highdef.png')

    sift = cv2.xfeatures2d.SURF_create()
    floor2_left_sift = sift.detectAndCompute(floor2_left, None)
    floor2_right_sift = sift.detectAndCompute(floor2_right, None)
    floor7_left_sift = sift.detectAndCompute(floor7_left, None)
    floor7_right_sift = sift.detectAndCompute(floor7_right, None)

    floor_sift_list = [floor2_left_sift, floor2_right_sift, floor7_left_sift, floor7_right_sift]

   # threshold_list = [90, 90, 15, 50]
    threshold_list = [50, 90, 15, 30]
    FloorDetector((floor_sift_list, sift, threshold_list))
    rospy.spin()


