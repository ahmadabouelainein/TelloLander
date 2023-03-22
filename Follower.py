
import cv2
import numpy as np
import time

import imutils
from djitellopy import Tello
import numpy as np
from itertools import cycle

(major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')

if __name__ == '__main__':
    width = 320
    height = 240

    # Connect to Tello
    me = Tello()
    me.connect()
    time.sleep(5)
    #me.takeoff()
    '''me.for_back_velocity = 0
    me.left_right_velocity = 0
    me.up_down_velocity = 0
    me.yaw_velocity = 0
    me.speed = 0'''

    print(me.get_battery())
    me.streamon()
    me.send_command_with_return("downvision 1")

    # Set up tracker.
    # Instead of MIL, you can also use

    tracker_types = ['BOOSTING', 'MIL', 'KCF', 'TLD', 'MEDIANFLOW', 'CSRT', 'MOSSE']
    tracker_type = tracker_types[2]

    if int(minor_ver) < 3:
        tracker = cv2.Tracker_create(tracker_type)
    else:
        if tracker_type == 'BOOSTING':
            tracker = cv2.Tracker_create('BOOSTING')
        if tracker_type == 'MIL':
            tracker = cv2.TrackerMIL_create()
        if tracker_type == 'KCF':
            tracker = cv2.TrackerKCF_create()
        if tracker_type == 'TLD':
            tracker = cv2.TrackerTLD_create()
        if tracker_type == 'MEDIANFLOW':
            tracker = cv2.TrackerMedianFlow_create()
        if tracker_type == 'CSRT':
            tracker = cv2.TrackerCSRT_create()
        if tracker_type == 'MOSSE':
            tracker = cv2.TrackerMOSSE_create()
# Read first frame.
    frame_read = me.get_frame_read()
    myFrame = frame_read.frame
    frame = cv2.rotate(myFrame, cv2.ROTATE_90_CLOCKWISE)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    #thresh = cv2.threshold(blurred, 40, 255, cv2.THRESH_BINARY)[1]
    thresh = cv2.threshold(blurred, 20,255, cv2.THRESH_BINARY)[1]
    inverted = cv2.bitwise_not(thresh)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    res = cv2.morphologyEx(inverted, cv2.MORPH_OPEN, kernel)
    res = cv2.morphologyEx(res, cv2.MORPH_CLOSE, kernel)
    res = cv2.morphologyEx(res, cv2.MORPH_CLOSE, kernel)
    cv2.imshow("Binary", res)
    cnts = cv2.findContours(res, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    #bbox = cv2.selectROI(frame, False)
    maxArea = 0
    maxC = cnts[0]
    for c in cnts:
        area = cv2.contourArea(c)
        if area > maxArea:
            maxC = c
            maxArea = area
    (x, y), radius = cv2.minEnclosingCircle(maxC)
    center = (int(x), int(y))
    radius = int(radius)
    length = int(np.sqrt(np.pi*radius*radius))
    x1 = center[0]-int(length/2)
    y1 = center[1]-int(length/2)
    x2 = int(length/2)+center[0]
    y2 = int(length/2)+center[1]
    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 5)
    cv2.imshow("Image", frame)
    bbox = [x1,y1,length,length]
    print(bbox)

    # Initialize tracker with first frame and bounding box
    ok = tracker.init(frame, bbox)
    refreshBox = False
    while True:
        # Read a new frame
        if refreshBox:
            tracker = cv2.TrackerKCF_create()
            ok = tracker.init(frame, bbox)
            refreshBox = False
        frame_read = me.get_frame_read()
        myFrame = frame_read.frame
        frame = cv2.rotate(myFrame, cv2.ROTATE_90_CLOCKWISE)
        #frame = cv2.resize(frame, (320, 240))

        # Start timer
        timer = cv2.getTickCount()

        # Update tracker

        ok, bbox = tracker.update(frame)

        # Calculate Frames per second (FPS)
        fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);
        # Draw bounding box
        if ok:
            # Tracking success
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, p1, p2, (255, 0, 255), 2, 1)
            cv2.circle(frame, (int((p1[0]+p2[0])/2), int((p1[1]+p2[1])/2)), radius=3, color=(255, 0, 255), thickness=-1)
            cv2.circle(frame, (120,160), radius=3, color=(0, 255, 0),thickness=-1)

            # Display tracker type on frame
            cv2.putText(frame, tracker_type + " Tracker", (100, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2);

            # Display FPS on frame
            cv2.putText(frame, "FPS : " + str(int(fps)), (100, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2);

            # Display result
            cv2.imshow("Tracking", frame)

            # Exit if ESC pressed
            k = cv2.waitKey(1) & 0xff
            if k == 27: break

            ## Following and Landing

            Xerr = int((p1[0] + p2[0]) / 2) - 120   #X distance to the center
            Yerr = -int((p1[1] + p2[1]) / 2) + 160  #Y distance to the center
            Kp = 1
            #y 19 | x 27 (platform y19 x19)
            Ysend=int(15*Yerr/((p1[1] + p2[1]) / 2)) #X distance to the center in cm
            Xsend=int(10*Xerr/((p1[0] + p2[0]) / 2)) #Y distance to the center in cm
            setSpeed = abs(int(Kp*(Xsend+Ysend)/2))
            print("Error in X: ", Ysend)
            print("Error in y: ", Xsend)
            print("Control input: ", setSpeed)
            print("Height: ", me.get_height())

            #print(me.get_height()) tracker brakes at height=20
            '''#me.go_xyz_speed(Ysend, -Xsend, -20, 10)
            if me.get_height() <= 20:               #Landing
                me.land()
            el'''
            thresh = 100*maxC/(me.get_height()+1)
            if abs(Xsend) <= 3 and abs(Ysend) <= 3:   #Descent
                me.send_rc_control(0, 0, -10, 0)
                print(me.get_height())
                myFrame = frame_read.frame
                frame = cv2.rotate(myFrame, cv2.ROTATE_90_CLOCKWISE)
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                blurred = cv2.GaussianBlur(gray, (5, 5), 0)
                # thresh = cv2.threshold(blurred, 40, 255, cv2.THRESH_BINARY)[1]
                thresh = cv2.threshold(blurred, 20, 255, cv2.THRESH_BINARY)[1]
                inverted = cv2.bitwise_not(thresh)
                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
                res = cv2.morphologyEx(inverted, cv2.MORPH_OPEN, kernel)
                res = cv2.morphologyEx(res, cv2.MORPH_CLOSE, kernel)
                res = cv2.morphologyEx(res, cv2.MORPH_CLOSE, kernel)
                cv2.imshow("Binary", res)
                cnts = cv2.findContours(res, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cnts = imutils.grab_contours(cnts)
                # bbox = cv2.selectROI(frame, False)
                maxArea = 0
                maxC = cnts[0]
                for c in cnts:
                    area = cv2.contourArea(c)
                    if area > maxArea:
                        maxC = c
                        maxArea = area
                (x, y), radius = cv2.minEnclosingCircle(maxC)
                center = (int(x), int(y))
                radius = int(radius)
                length = int(np.sqrt(np.pi * radius * radius))
                x1 = center[0] - int(length / 2)
                y1 = center[1] - int(length / 2)
                x2 = int(length / 2) + center[0]
                y2 = int(length / 2) + center[1]
                bbox = [x1, y1, length, length]
                refreshBox = True
            else:
                me.send_rc_control(int(Xsend), int(Ysend), 0, 0)    #Following

            '''if abs(Xsend)<=20 and abs(Ysend)<=20:
                me.stop
            else:
                me.go_xyz_speed(Ysend, Xsend, -1, 10)
            '''

            '''print("Error in X: ", Xerr)
                    print("Error in y: ", Yerr)
                    print("Control input: ", setSpeed)'''

        else:
            # Tracking failure
            cv2.putText(frame, "Tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
            me.for_back_velocity = 0
            me.left_right_velocity = 0
            me.up_down_velocity = 0
            me.yaw_velocity = 0
            me.speed = 0

        '''Xerr = int((p1[0]+p2[0])/2)-120
        Yerr = int((p1[1]+p2[1])/2)-160'''

        # Display tracker type on frame
        cv2.putText(frame, tracker_type + " Tracker", (100, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2);

        # Display FPS on frame
        cv2.putText(frame, "FPS : " + str(int(fps)), (100, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2);

        # Display result
        cv2.imshow("Tracking", frame)

        # Exit if ESC pressed
        k = cv2.waitKey(1) & 0xff
        if k == 27: break
    me.end()
    cv2.destroyAllWindows()