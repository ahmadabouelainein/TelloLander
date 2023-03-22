from turtle import width
from djitellopy import Tello
import cv2
import time


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
'''
me.takeoff()
time.sleep(8)
me.rotate_clockwise(90)
time.sleep(3)
me.rotate_counter_clockwise(90)
time.sleep(3)
'''
# me.streamoff()

t = time.time()
c = 0
while c <= 20:
    # extract image
    frame_read = me.get_frame_read()
    myFrame = frame_read.frame
    img = cv2.resize(myFrame, (width, height))
    cv2.imshow("MyResult", img)

    if cv2.waitKey(1) & 0XFF == ord('q'):
        me.land()
        break
cv2.destroyAllWindows()
time.sleep(3)
me.land()
