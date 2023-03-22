
import cv2
import time
import resource
import imutils
from djitellopy import Tello
import numpy as np

(major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')


def tracker_create(tracker_type):
    """
    Function creates a tracker based on the type passed as a parameter
    Students should create a multi-purpose function for creating a tracker of any type , they will recreate full tracker_create function
    (students could decide which tracker to use and expalin why, but should write a function tthat allows a creation of any type)
    """
    #############
    ...
    #############
    return


def read_frame(raw_frame):
    """
    Function uses a frame captures by drone, and creates a bounding box of a landing pad to follow it
    Students should complete the function by writing  a transformation of the area countuers to a bounding box
    """
    myFrame = raw_frame.frame
    frame = cv2.rotate(myFrame, cv2.ROTATE_90_CLOCKWISE)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    thresh = cv2.threshold(blurred, ..., 255, cv2.THRESH_BINARY)[1]  ############# Choose the threshld parameter
    inverted = cv2.bitwise_not(thresh)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    res = cv2.morphologyEx(inverted, cv2.MORPH_OPEN, kernel)
    res = cv2.morphologyEx(res, cv2.MORPH_CLOSE, kernel)
    res = cv2.morphologyEx(res, cv2.MORPH_CLOSE, kernel)
    cv2.imshow("Binary", res)
    cnts = cv2.findContours(res, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)  #Countuers of areas
    #############
    ...
    bbox = ...
    #############
    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 5)
    cv2.imshow("Image", frame)
    return frame, bbox


if __name__ == '__main__':
    width = 320
    height = 240

    ############# Students should write a code to connect and start a drone and turn on downward ccameras
    ...
    ############# Create a tracker using a function above
    tracker = tracker_create(...)
    #############

# Read first frame.
    raw_frame = ... ############# Read a frame from the drone
    frame, bbox = read_frame(raw_frame)
    print(bbox)

    # Initialize tracker with first frame and bounding box
    ok = tracker.init(frame, bbox)
    # Main Loop
    while True:
        # Read a new frame
        ############# Students should write a condition, such that tracker would be recreated every time on bounding box updates
        ...
        #############

        # Start timer
        timer = cv2.getTickCount()

        # Update tracker
        ok, bbox = tracker.update(frame)

        # Calculate Frames per second (FPS)
        fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);
        if ok:
            # Tracking success
            # Draw bounding box
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

            ############# Students should write a function responsible for drone following and landing
            ############# During descent, the bounding box has to be recalculated
            ...
            #############
        else:
            # Tracking failure
            #############  Students should implement a failure behaviour
            cv2.putText(frame, "Tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
            ...
            #############

            # Display tracker type on frame
            cv2.putText(frame, tracker_type + " Tracker", (100, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2);

            # Display FPS on frame
            cv2.putText(frame, "FPS : " + str(int(fps)), (100, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2);

            # Display result
            cv2.imshow("Tracking", frame)

            # Exit if ESC pressed
            k = cv2.waitKey(1) & 0xff
            if k == 27: break

    ... ############# Stop and shutdown the drone
    cv2.destroyAllWindows()