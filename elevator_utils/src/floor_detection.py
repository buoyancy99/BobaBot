#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int8

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import copy


to_detect = 0


def fm(floor_sift, img2, sift, MIN_MATCH_COUNT=15):

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

        print('Floor!')
        return 1
    else:
        print("Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT))
        return 0


def to_detect_callback(msg):
    to_detect = msg.data

def color_image_callback(img_msg, args):
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e))


    #cv2.imshow("door_detector", cv_image)

    global to_detect
    
    if to_detect == 0:
        return

    floor_sift_list = args[0]
    floor_sift = floor_sift_list[to_detect-1]
    sift = args[1]
    floor_value = fm(floor_sift, cv_image, sift)
    pub.publish(floor_value)

    k = cv2.waitKey(3)



def run_detection(args):
    pub = rospy.Publisher('floor_value', Int8, queue_size=10)
    rospy.init_node('floor_detector')
    rate = rospy.Rate(10)

    bridge = CvBridge()

    rospy.Subscriber("floor_to_detect", Int8, to_detect_callback)
    
    color_image = rospy.Subscriber("/mynteye/left/image_color", Image, color_image_callback, args)

    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()

if __name__ == "__main__":
    floor2_left = cv2.imread('/home/boyuan/Projects/BobaBot/src/elevator_utils/src/floor2_left.png')
    floor2_right = cv2.imread('/home/boyuan/Projects/BobaBot/src/elevator_utils/src/floor2_right.png')
    floor7_left = cv2.imread('/home/boyuan/Projects/BobaBot/src/elevator_utils/src/floor7_left_fix.png')
    floor7_right = cv2.imread('/home/boyuan/Projects/BobaBot/src/elevator_utils/src/floor7_right.png')

    sift = cv2.xfeatures2d.SURF_create()
    floor2_left_sift.detectAndCompute(floor2_left, None)
    floor2_right_sift.detectAndCompute(floor2_right, None)
    floor7_left_sift.detectAndCompute(floor7_left, None)
    floor7_right_sift.detectAndCompute(floor7_right, None)

    floor_sift_list = [floor2_left_sift, floor2_right_sift, floor7_left_sift, floor7_right_sift]

    run_detection((floor_sift_list, sift))


