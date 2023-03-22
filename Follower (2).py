
import cv2
import time
#import resource
import imutils
from djitellopy import Tello
import numpy as np

(major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')


'''def memory_limit():
    soft, hard = resource.getrlimit(resource.RLIMIT_AS)
    resource.setrlimit(resource.RLIMIT_AS, (get_memory() * 1024, hard))
    soft, hard = resource.getrlimit(resource.RLIMIT_AS)
    print('Memory:',soft)


def get_memory():
    with open('/proc/meminfo', 'r') as mem:
        free_memory = 0
        for i in mem:
            sline = i.split()
            #if str(sline[0]) in ('MemFree:', 'Buffers:', 'Cached:'):   #Limit Free RAM
            if str(sline[0]) in ('MemTotal:'):                          # Limit Total RAM
                free_memory += int(sline[1])
    return free_memory'''


def tracker_create(tracker_type):
    """
    This function creates a tracker based on the type passed as a parameter
    Students should create a multi-purpose function for creating a tracker of any type , they will recreate full tracker_create function
    (students could decide which tracker to use and expalin why, but should write a function tthat allows a creation of any type)
    """
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
    return tracker


def read_frame(frame_read):
    """
    Function uses a frame captures by drone, and creates a bounding box of a landing pad to follow it
    """
    myFrame = frame_read.frame
    frame = cv2.rotate(myFrame, cv2.ROTATE_90_CLOCKWISE)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    thresh = cv2.threshold(blurred, 20, 255, cv2.THRESH_BINARY)[1]  ############# We can remove thresholding values so that students would find them themselves and explain why they chose them
    inverted = cv2.bitwise_not(thresh)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    res = cv2.morphologyEx(inverted, cv2.MORPH_OPEN, kernel)
    res = cv2.morphologyEx(res, cv2.MORPH_CLOSE, kernel)
    res = cv2.morphologyEx(res, cv2.MORPH_CLOSE, kernel)
    cv2.imshow("Binary", res)
    cnts = cv2.findContours(res, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)  #Countuers of areas
    ############# Write a function of finding a landing pad
    ############# Students should identify that the landing pad would be the biggest area and write a function to find a biggest area within countuers using cv2.contourArea(c) function
    maxArea = 0
    maxC = cnts[0]
    for c in cnts:
        area = cv2.contourArea(c)
        if area > maxArea:
            maxC = c
            maxArea = area
    #############
    (x, y), radius = cv2.minEnclosingCircle(maxC)
    ############# Write a function of transforming a bounding circle to a bounding box
    ############# We can also ask students to find a function cv2.minEnclosingCircle(maxC), to find bounding circle as well
    center = (int(x), int(y))
    radius = int(radius)
    area = np.pi * radius * radius
    length = int(np.sqrt(area))
    x1 = center[0] - int(length / 2)
    y1 = center[1] - int(length / 2)
    x2 = int(length / 2) + center[0]
    y2 = int(length / 2) + center[1]
    bbox = [x1, y1, length, length]
    #############
    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 5)
    cv2.imshow("Image", frame)
    return frame, bbox, maxArea


if __name__ == '__main__':
    width = 320
    height = 240
    #memory_limit() #Limit the RAM
    ############# Students should write a code to connect and start a drone and turn on downward ccameras
    ############# During previous labs they should try running some simple commands such move_forward() and etc., to familiarize with drone.
    # Connect to Tello
    drone = Tello()
    drone.connect()
    time.sleep(5)
    #drone.takeoff()

    print(drone.get_battery())
    drone.streamon()
    drone.send_command_with_return("downvision 1")
    #############
    #drone.move_up(20)
    #time.sleep(2)
    ############# Create a tracker using a function that they used (Maybe they could explain what is the purpose of functions in Python)
    # Set up tracker.
    # Instead of KCF, you can also try others
    tracker_types = ['BOOSTING', 'MIL', 'KCF', 'TLD', 'MEDIANFLOW', 'CSRT', 'MOSSE']
    tracker_type = tracker_types[2]
    tracker = tracker_create(tracker_type)
    #############

# Read first frame.
    frame_read = drone.get_frame_read()
    frame, bbox, area = read_frame(frame_read)
    print(bbox)
    h = 0
    # Initialize tracker with first frame and bounding box
    ok = tracker.init(frame, bbox)
    refreshBox = False
    # Main Loop
    while True:
        # Read a new frame
        ############# Students should write an update of tracker (maybe could be kept by us)
        ############# Students would need to write a modification, such that tracker would be redone every time on descent
        ############# (becuase on descent a bounding box is recalculated for which a new instance of a tracker is needed)
        if refreshBox: # If bounding box was recalculated, reinitialize the tracker
            tracker = tracker_create(tracker_type)
            ok = tracker.init(frame, bbox)
            refreshBox = False
        frame_read = drone.get_frame_read()
        myFrame = frame_read.frame
        frame = cv2.rotate(myFrame, cv2.ROTATE_90_CLOCKWISE)

        # Start timer
        timer = cv2.getTickCount()

        # Update tracker
        ok, bbox = tracker.update(frame)
        #############  Function to create windows with FPS and drawn bounding box woth centers (maybe unnessecary to ask but we could ask to add visualization)
        # Calculate Frames per second (FPS)
        fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
        if ok:
            # Tracking success
            # Draw bounding box
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, p1, p2, (255, 0, 255), 2, 1)
            cv2.circle(frame, (int((p1[0]+p2[0])/2), int((p1[1]+p2[1])/2)), radius=3, color=(255, 0, 255), thickness=-1)
            cv2.circle(frame, (120,160), radius=3, color=(0, 255, 0),thickness=-1)

            # Display tracker type on frame
            cv2.putText(frame, tracker_type + " Tracker", (100, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)

            # Display FPS on frame
            cv2.putText(frame, "FPS : " + str(int(fps)), (100, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)

            # Display result
            cv2.imshow("Tracking", frame)

            # Exit if ESC pressed
            k = cv2.waitKey(1) & 0xff
            if k == 27: break
            #############

            ######################## Students should write a function for following and landing
            #############  they could decide how to achieve that themselves without our hints from the start
            # Following and Landing

            Xerr = int((p1[0] + p2[0]) / 2) - 120   #X distance to the center
            Yerr = -int((p1[1] + p2[1]) / 2) + 160  #Y distance to the center

            Ysend=int(13*Yerr/((p1[1] + p2[1]) / 2)) #X distance to the center in cm
            Xsend=int(13*Xerr/((p1[0] + p2[0]) / 2)) #Y distance to the center in cm
            print("Error in X: ", Ysend)
            print("Error in y: ", Xsend)
            print("Area: ", area)
            h = drone.get_height()  # + h
            print("height: ", h)
            thresh = (area/1000)/(h+1)
            print("Threshold: ", thresh)
            '''if drone.get_height() <= 30:               #Landing
                drone.land()'''
            if abs(Xsend) <= int(thresh) and abs(Ysend) <= int(thresh):   #Descent
                drone.send_rc_control(0, 0, -10, 0)
                print("Height: ", drone.get_height())
                frame, bbox, area = read_frame(frame_read)
                refreshBox = True
            else:
                drone.send_rc_control(int(Xsend), int(Ysend), 0, 0)    #Following
            ########################
        else:
            # Tracking failure
            #############  Students could do something when the tracker failed, depending on their implementation of followingand landing functions
            #############  we set the speed to 0, but drone would not immeditialy stop and would finish his previous commands with inertia, which would bring it back to landing pad
            cv2.putText(frame, "Tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
            drone.for_back_velocity = 0
            drone.left_right_velocity = 0
            drone.up_down_velocity = 0
            drone.yaw_velocity = 0
            drone.speed = 0
            #############

            #############  Function to create windows with FPS and drawn bounding box woth centers (maybe unnessecary to ask but we could ask to add visualization)
            # Display tracker type on frame
            cv2.putText(frame, tracker_type + " Tracker", (100, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)

            # Display FPS on frame
            cv2.putText(frame, "FPS : " + str(int(fps)), (100, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)

            # Display result
            cv2.imshow("Tracking", frame)

            # Exit if ESC pressed
            k = cv2.waitKey(1) & 0xff
            if k == 27: break
            #############
    drone.end() #############  Command to end the drone, would be removed at the end
    cv2.destroyAllWindows()