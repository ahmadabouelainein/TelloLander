
import cv2
import sys
import time
from djitellopy import Tello
import numpy as np

(major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')

if __name__ == '__main__':
    width = 320
    height = 240

    # Connect to Tello
    me = Tello()
    me.connect()
    time.sleep(8)

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
    #frame = cv2.resize(frame,(320,240))
    # Define an initial bounding box
    #bbox = (287, 23, 86, 320)
    # Uncomment the line below to select a different bounding box
    bbox = cv2.selectROI(frame, False)
    # Initialize tracker with first frame and bounding box
    ok = tracker.init(frame, bbox)
    while True:
        # Read a new frame
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
        else:
            # Tracking failure
            cv2.putText(frame, "Tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
        Xerr = int((p1[0]+p2[0])/2)-120
        Yerr = int((p1[1]+p2[1])/2)-160
        print("Error in X: ", Xerr)
        print("Error in y: ", Yerr)
        # Display tracker type on frame
        cv2.putText(frame, tracker_type + " Tracker", (100, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2);

        # Display FPS on frame
        cv2.putText(frame, "FPS : " + str(int(fps)), (100, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2);

        # Display result
        cv2.imshow("Tracking", frame)

        # Exit if ESC pressed
        k = cv2.waitKey(1) & 0xff
        if k == 27: break
