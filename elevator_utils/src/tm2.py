
import sys
import numpy as np
import time
import cv2
#import imutils

major_ver, minor_ver, subminor_ver = (cv2.__version__).split('.')
print(major_ver, minor_ver, subminor_ver)

def optic_flow(frame1, frame2):
    #ret, frame1 = cap.read()
    prvs = cv2.cvtColor(frame1,cv2.COLOR_BGR2GRAY)
    hsv = np.zeros_like(frame1)
    hsv[...,1] = 255

    #while(1):
    #ret, frame2 = cap.read()
    next = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY)

    flow = cv2.calcOpticalFlowFarneback(prvs,next, None, 0.5, 3, 15, 3, 5, 1.2, 0)
    if flow.max() < 5:
        return
    print(flow.shape)
    print(np.argmax(flow))
    print(np.argmin(flow))

    mag, ang = cv2.cartToPolar(flow[...,0], flow[...,1])
    hsv[...,0] = ang*180/np.pi/2
    hsv[...,2] = cv2.normalize(mag,None,0,255,cv2.NORM_MINMAX)
    rgb = cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)

    cv2.imshow('frame2',rgb)
    k = cv2.waitKey(30) & 0xff
    #if k == 27:
    #    break
    if k == ord('s'):
        cv2.imwrite('opticalfb.png',frame2)
        cv2.imwrite('opticalhsv.png',rgb)
    #prvs = next

MIN_MATCH_COUNT = 10

def fm(img1, img2):
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
        print(len(good))
        src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
        #matchesMask = mask.ravel().tolist()

        h,w = img1[:,:,0].shape
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        dst = cv2.perspectiveTransform(pts,M)

        print(dst)

        img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)

    else:
        print("Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT))
        #matchesMask = None

    #print(mask.shape)

def edge(img):
    #return cv2.Canny(img, 0, 100)
    #return cv2.GaussianBlur(img,(5,5),0)
    #return cv2.equalizeHist(img) 
    return img

def tm(img, template, prev_loc=None):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
    #template = template[:,:,0]
    #print(template.shape)
    cv2.imshow("bbox", template)
    w, h = template.shape[::-1]
    method = eval('cv2.TM_SQDIFF')

    # Apply template Matching
    start_time = time.time()
    #res = cv2.matchTemplate(img2,template,method)
    #min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
    #print("--- %s seconds ---" % (time.time() - start_time))


    #if min_val > 450000.0:
    #    return


    found = None
    for scale in np.linspace(0.2, 1.0, 20)[::-1]:
        # resize the image according to the scale, and keep track
        # of the ratio of the resizing
        resized = imutils.resize(gray, width = int(gray.shape[1] * scale))
        resized = edge(resized)
        r = gray.shape[1] / float(resized.shape[1])
 
        # if the resized image is smaller than the template, then break
        # from the loop
        if resized.shape[0] < h or resized.shape[1] < w:
            break

        result = cv2.matchTemplate(resized, template, method)
        (min_val, max_val, min_loc, max_loc) = cv2.minMaxLoc(result)
 
        # check to see if the iteration should be visualized
        if False:
            # draw a bounding box around the detected region
            clone = np.dstack([edged, edged, edged])
            cv2.rectangle(clone, (maxLoc[0], maxLoc[1]),
                (maxLoc[0] + tW, maxLoc[1] + tH), (0, 0, 255), 2)
            cv2.imshow("Visualize", clone)
            cv2.waitKey(0)
 
        # if we have found a new maximum correlation value, then update
        # the bookkeeping variable
        if found is None or min_val < found[0]:
            found = (min_val, min_loc, r)


    # # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take minimum
    # if method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
    #     top_left = min_loc
    # else:
    #     top_left = max_loc
    # bottom_right = (top_left[0] + w, top_left[1] + h)

    # print(top_left, bottom_right)

    # cv2.rectangle(img,top_left, bottom_right, 255, 2)
    #return img

    (v, maxLoc, r) = found
    
    (startX, startY) = (int(maxLoc[0] * r), int(maxLoc[1] * r))
    (endX, endY) = (int((maxLoc[0] + w) * r), int((maxLoc[1] + h) * r))

    print("--- %s seconds ---" % (time.time() - start_time))
    print(v)
    # if prev_loc and abs(maxLoc[0] - prev_loc[0]) > 10 or v > 5000000.0:
    #     return maxLoc
    if v > 45589504.0:
        return

    

    #img[img[:,:,0] < 150] = 0

    #img = img[img[:,:,0] > 150]
 
    # draw a bounding box around the detected result and display the image
    cv2.rectangle(img, (startX, startY), (endX, endY), (0, 0, 255), 2)
    print(endX-startX)
    return maxLoc


def tm_multi(img, template_list, prev_loc=None):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #template = template[:,:,0]
    #print(template.shape)
    cv2.imshow("bbox", template)
    w, h = template[0].shape[::-1]
    method = eval('cv2.TM_SQDIFF')

    # Apply template Matching
    start_time = time.time()
    #res = cv2.matchTemplate(img2,template,method)
    #min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
    #print("--- %s seconds ---" % (time.time() - start_time))


    #if min_val > 450000.0:
    #    return


    found = None
    for template in template_list:
        for scale in np.linspace(0.2, 1.0, 20)[::-1]:
            # resize the image according to the scale, and keep track
            # of the ratio of the resizing
            resized = imutils.resize(gray, width = int(gray.shape[1] * scale))
            resized = edge(resized)
            r = gray.shape[1] / float(resized.shape[1])
     
            # if the resized image is smaller than the template, then break
            # from the loop
            if resized.shape[0] < h or resized.shape[1] < w:
                break

            result = cv2.matchTemplate(resized, template, method)
            (min_val, max_val, min_loc, max_loc) = cv2.minMaxLoc(result)
     
            # check to see if the iteration should be visualized
            if False:
                # draw a bounding box around the detected region
                clone = np.dstack([edged, edged, edged])
                cv2.rectangle(clone, (maxLoc[0], maxLoc[1]),
                    (maxLoc[0] + tW, maxLoc[1] + tH), (0, 0, 255), 2)
                cv2.imshow("Visualize", clone)
                cv2.waitKey(0)
     
            # if we have found a new maximum correlation value, then update
            # the bookkeeping variable
            if found is None or min_val < found[0]:
                found = (min_val, min_loc, r)


    # # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take minimum
    # if method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
    #     top_left = min_loc
    # else:
    #     top_left = max_loc
    # bottom_right = (top_left[0] + w, top_left[1] + h)

    # print(top_left, bottom_right)

    # cv2.rectangle(img,top_left, bottom_right, 255, 2)
    #return img

    (v, maxLoc, r) = found
    print(v)
    (startX, startY) = (int(maxLoc[0] * r), int(maxLoc[1] * r))
    (endX, endY) = (int((maxLoc[0] + w) * r), int((maxLoc[1] + h) * r))

    print("--- %s seconds ---" % (time.time() - start_time))

    #if prev_loc and abs(maxLoc[0] - prev_loc[0]) > 10 and v > 400000.0:
    #    return

    img[img[:,:,0] < 150] = 0

    #img = img[img[:,:,0] > 150]
 
    # draw a bounding box around the detected result and display the image
    cv2.rectangle(img, (startX, startY), (endX, endY), (0, 0, 255), 2)
    
    return maxLoc

 
if __name__ == '__main__':
 
    # Set up tracker.
    # Instead of MIL, you can also use

    #button = cv2.imread('button.png')[:,:,0]
 
    # Read video
    video = cv2.VideoCapture(0)
    
 
    # # Exit if video not opened.
    # if not video.isOpened():
    #     print("Could not open video")
    #     sys.exit()
 
    # # Read first frame.
    # for i in range(5):    
    #     ok, frame = video.read()
    #     if not ok:
    #         break
    # # #print(np.count_nonzero(np.array(frame[:,:,0]>200)))
    # cv2.imshow("Tracking", frame)
     
    # # # Define an initial bounding box
    # # bbox = (287, 23, 86, 320)
 
    # # # Uncomment the line below to select a different bounding box
    # r = cv2.selectROI(frame, False)
    # #button = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # button = frame[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])]
    button = cv2.imread('elevator_utils/src/elevator_inside_02.png')
    button = edge(button)
    button = cv2.resize(button, (button.shape[0]//5, button.shape[1]//5 ))
    cv2.imshow("bbox", button)
    print(button.shape)
 
    # # Initialize tracker with first frame and bounding box
    # ok = tracker.init(frame, bbox)
 
    prev_loc = None
    ok, prev_frame = video.read()
    i=0
    while True:
        # Read a new frame
        ok, frame = video.read()
        if not ok:
            break
         
        # Start timer
        timer = cv2.getTickCount()
 
        # Update tracker
        #ok, bbox = tracker.update(frame)
 
        # Calculate Frames per second (FPS)
        fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);
 
        # Draw bounding box
        if ok:
            # Tracking success
            # p1 = (int(bbox[0]), int(bbox[1]))
            # p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            # cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
            if i % 20 == 0:
                optic_flow(prev_frame, frame)
                prev_frame = frame
            i+=1
            #prev_loc = tm(frame, button, prev_loc)
            #try:
            #    fm(button, frame)
            #except:
            #    continue
            
        else :
            # Tracking failure
            cv2.putText(frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
 
        # Display tracker type on frame
        #cv2.putText(frame, tracker_type + " Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2);
     
        # Display FPS on frame
        #cv2.putText(frame, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);
 
        # Display result
        cv2.imshow("Tracking", frame)
 
        # Exit if ESC pressed
        k = cv2.waitKey(1) & 0xff
        if k == 27 : break