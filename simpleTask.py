from djitellopy import Tello
import time
tello = Tello()

tello.connect()
time.sleep(5)
#tello.takeoff()
t = time.time()

while time.time()-t<20:
    print(tello.get_height())
    print(tello.get_)
    print("======")
#tello.land()
'''time.sleep(8)
tello.move_left(1)
time.sleep(3)
tello.rotate_clockwise(90)
time.sleep(3)
tello.move_forward(100)
time.sleep(3)
tello.land()'''