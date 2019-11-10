import cv2
import sys
import numpy as np
import time
import imutils
 
major_ver, minor_ver, subminor_ver = (cv2.__version__).split('.')
print(major_ver, minor_ver, subminor_ver)

def tm(img, template):
    gray = img[:,:,0]
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
    for scale in np.linspace(0.2, 1.5, 5)[::-1]:
        # resize the image according to the scale, and keep track
        # of the ratio of the resizing
        resized = imutils.resize(gray, width = int(gray.shape[1] * scale))
        r = gray.shape[1] / float(resized.shape[1])
 
        # if the resized image is smaller than the template, then break
        # from the loop
        if resized.shape[0] < h or resized.shape[1] < w:
            break

        result = cv2.matchTemplate(gray, template, method)
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

    (_, maxLoc, r) = found
    (startX, startY) = (int(maxLoc[0] * r), int(maxLoc[1] * r))
    (endX, endY) = (int((maxLoc[0] + w) * r), int((maxLoc[1] + h) * r))

    print("--- %s seconds ---" % (time.time() - start_time))
 
    # draw a bounding box around the detected result and display the image
    cv2.rectangle(img, (startX, startY), (endX, endY), (0, 0, 255), 2)

 
if __name__ == '__main__' :
 
    # Set up tracker.
    # Instead of MIL, you can also use

    #button = cv2.imread('button.png')[:,:,0]
 
    # Read video
    video = cv2.VideoCapture(1)
 
    # Exit if video not opened.
    if not video.isOpened():
        print("Could not open video")
        sys.exit()
 
    # Read first frame.
    for i in range(5):    
        ok, frame = video.read()
        if not ok:
            break
    # #print(np.count_nonzero(np.array(frame[:,:,0]>200)))
    cv2.imshow("Tracking", frame)
     
    # # Define an initial bounding box
    # bbox = (287, 23, 86, 320)
 
    # # Uncomment the line below to select a different bounding box
    r = cv2.selectROI(frame, False)
    button = frame[:,:,0][int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])]
    #cv2.imshow("bbox", button)
    print(button.shape)
 
    # # Initialize tracker with first frame and bounding box
    # ok = tracker.init(frame, bbox)
 
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
            tm(frame, button)
        else :
            # Tracking failure
            cv2.putText(frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
 
        # Display tracker type on frame
        #cv2.putText(frame, tracker_type + " Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2);
     
        # Display FPS on frame
        cv2.putText(frame, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);
 
        # Display result
        cv2.imshow("Tracking", frame)
 
        # Exit if ESC pressed
        k = cv2.waitKey(1) & 0xff
        if k == 27 : break