#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int8

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import copy

max_depth_loc = None
img_buffer = np.zeros((4, 720, 1280))
img_count = 0
prev_frame = []

MIN_MATCH_COUNT = 5
marker0 = cv2.imread('~/Projects/BobaBot/src/elevator_utils/src/marker0.png')

# Generate contours to detect corners of the tag
def contour_generator(frame):
    test_img1 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    test_blur = cv2.GaussianBlur(test_img1, (5, 5), 0)
    edge = cv2.Canny(test_blur, 75, 200)
    edge1 = copy.copy(edge)
    contour_list = list()

    r, cnts, h = cv2.findContours(edge1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    index = list()
    for hier in h[0]:
        if hier[3] != -1:
            index.append(hier[3])

    # loop over the contours
    for c in index:
        peri = cv2.arcLength(cnts[c], True)
        approx = cv2.approxPolyDP(cnts[c], 0.02 * peri, True)

        if len(approx) > 4:
            peri1 = cv2.arcLength(cnts[c - 1], True)
            corners = cv2.approxPolyDP(cnts[c - 1], 0.02 * peri1, True)
            contour_list.append(corners)

    new_contour_list = list()
    for contour in contour_list:
        if len(contour) == 4:
            new_contour_list.append(contour)
    final_contour_list = list()
    for element in new_contour_list:
        if cv2.contourArea(element) < 2500:
            final_contour_list.append(element)

    return final_contour_list

def run_detection(args):
    pub = rospy.Publisher('elevator_door_open', Int8, queue_size=10)
    pub_floor2 = rospy.Publisher('floor2_value', Int8, queue_size=10)
    pub_floor7 = rospy.Publisher('floor7_value', Int8, queue_size=10)
    rospy.init_node('elevator_detector')
    #rate = rospy.Rate(10)

    bridge = CvBridge()
    cv2.namedWindow("door_detector", 0)


    def optic_flow(frame2):
        global prev_frame
        global img_count
        if len(prev_frame)==0:
            prev_frame = frame2
            return
        img_count += 1
        if img_count % 20 != 0:
            return

        frame1 = prev_frame
        #ret, frame1 = cap.read()
        prvs = cv2.cvtColor(frame1,cv2.COLOR_BGR2GRAY)
        hsv = np.zeros_like(frame1)
        hsv[...,1] = 255

        #while(1):
        #ret, frame2 = cap.read()
        next = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY)

        flow = cv2.calcOpticalFlowFarneback(prvs,next, None, 0.5, 3, 15, 3, 5, 1.2, 0)
        if flow.max() < 20:
            return 0 # no door
        #print(flow.max())
        #print(flow.shape)
        
        #print(np.argmax(flow))
        #print(np.argmin(flow))
        #print(argmax2d(flow))

        mag, ang = cv2.cartToPolar(flow[...,0], flow[...,1])
        hsv[...,0] = ang*180/np.pi/2
        hsv[...,2] = cv2.normalize(mag,None,0,255,cv2.NORM_MINMAX)
        rgb = cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)
        cv2.circle(rgb, argmax2d(rgb[:,:,0]),  20, (0, 255, 0), -1)
        #ret,rgb = cv2.threshold(rgb,127,255,cv2.THRESH_BINARY)
        print(argmax2d(rgb[:,:,0]))
        #rgb = rgb[:,:,0]
        

        cv2.imshow('optic_flow',rgb)
        k = cv2.waitKey(30) & 0xff
        #if k == 27:
        #    breakmarker0
        if k == ord('s'):
            cv2.imwrite('opticalfb.png',frame2)
            cv2.imwrite('opticalhsv.png',rgb)
        #prvs = next

        prev_frame = frame2
        if argmax2d(rgb[:,:,0])[0] < 1280/2:
            return 1 #left door
        else:
            return 2 #rigth door  

    def detect_max_increase_depth(img_now, img2_prev):
        diff = img1 - img2
        return argmax2d(diff)
    def bbox(img):
        #r = cv2.selectROI(img, False)
        r = (100, 100, 300, 500)
        #print(r)
        #cv2.rectangle(img, (startX, startY), (endX, endY), (0, 0, 255), 2)
        return img[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])]
        
    def blur_avg(img):
        #return cv2.Canny(img, 0, 100)
        return cv2.GaussianBlur(img,(11,11),10)
        #return cv2.equalizeHist(img) 
        return img

    def img_fill(im_in, n):  # n = binary image threshold
        th, im_th = cv2.threshold(im_in, n, 255, cv2.THRESH_BINARY)

        # Copy the thresholded image.
        im_floodfill = im_th.copy()

        # Mask used to flood filling.
        # Notice the size needs to be 2 pixels than the image.
        h, w = im_th.shape[:2]
        mask = np.zeros((h + 2, w + 2), np.uint8)

        # Floodfill from point (0, 0)
        cv2.floodFill(im_floodfill, mask, (210, 700), 255)

        # Invert floodfilled image
        im_floodfill_inv = cv2.bitwise_not(im_floodfill)

        # Combine the two images to get the foregroufm(marker0, cv_image)nd.
        fill_image = im_th | im_floodfill_inv

        return fill_image 

    def argmax2d(X):
        n, m = X.shape
        x_ = np.ravel(X)
        k = np.argmax(x_)
        i, j = k // m, k % m
        return j, i



    def fm(img1, img2):
        global MIN_MATCH_COUNT
        # Initiate SIFT detector
        sift = cv2.xfeatures2d.SURF_create()

        # find the keypoints and descriptors with SIFT
        kp1, des1 = sift.detectAndCompute(img1,None)
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
            #matchesMask = None
            return 0

    def image_callback(img_msg):
        try:
            cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        cv2.imshow("door_detector", cv_image)
        k = cv2.waitKey(3)

    def depth_image_callback(img_msg):
        try:
            cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        global max_depth_loc
        #print(cv_image.shape)
        cv_image = cv_image / 16384.0
        img_buffer[:-1] = img_buffer[1:]
        img_buffer[-1] = cv_image
        cv_image = img_buffer[0] * 0.1 + img_buffer[1] * 0.2 + img_buffer[2] * 0.3 + img_buffer[3] * 0.4
        cv_image = (cv_image * 255.0).astype(np.uint8)
        cv_image = blur_avg(cv_image)
        max_depth_loc = argmax2d(cv_image)

        cv2.imshow("door_depth", cv_image)

    def color_image_callback(img_msg, args):
        try:
            cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        #print(cv_image.shape)
        #max_depth_loc = argmax2d(cv_image)
        #print(max_depth_loc)
        #cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)
        #cv_image = np.zeros(cv_image.shape)
        door_value = None
        #door_value = optic_flow(cv_image)
        
        if door_value is not None:
            print(door_value)
            pub.publish(door_value)

        #print(max_depth_loc)
        #cv2.imshow("cropped", bbox(cv_image))
        #cv2.circle(cv_image, max_depth_loc, 50, (0, 255, 0), -1)
        # print(cv_image.shape)
        #cv_image = img_fill(cv_image, 150)
        cv2.imshow("door_detector", cv_image)
        marker0 = args[0]
        marker0_2 = args[1]
        fm(marker0_2, cv_image)
        #floor_value = max(fm(marker0, cv_image), fm(marker0_2, cv_image))
        #print(floor_value)
        # r = cv2.selectROI(cv_image, False)
        # #button = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # marker0 = cv_image[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])]
        # cv2.imwrite( 'mynteye_marker0_3.png', marker0)
        # #print(len(contour_generator(marker0)))
        #pub_floor2.publish(floor_value)
        #pub_floor7.publish(fm(marker7, cv_image))

        
        k = cv2.waitKey(3)

    #cv2.setMouseCallback('TeleOp', mouse_callback)
    #sub_image = rospy.Subscriber("/mynteye/left/image_color", Image, image_callback)

    #d = rospy.Subscriber("/mynteye/depth/image_raw", Image, depth_image_callback)
    
    color_image = rospy.Subscriber("/mynteye/left/image_color", Image, color_image_callback, args)


    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == "__main__":
    marker0 = cv2.imread('/home/boyuan/Projects/BobaBot/src/elevator_utils/src/mynteye_marker0_2.png')
    marker0_2 = cv2.imread('/home/boyuan/Projects/BobaBot/src/elevator_utils/src/mynteye_marker0_3.png')
    print(marker0.shape)
    run_detection((marker0, marker0_2))