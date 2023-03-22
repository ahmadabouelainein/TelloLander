#from tracker import *
import cv2

cap = cv2.VideoCapture("MovingCam.mp4")

# Object detection from Stable camera
object_detector = cv2.createBackgroundSubtractorMOG2()

while True:
    ret, frame = cap.read()
    frame = cv2.resize(frame,(960,540))
    # 1. Object Detection
    mask = object_detector.apply(frame)
    cv2.imshow("mask", mask)
    _, mask = cv2.threshold(mask, 254, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        # Calculate area and remove small elements
        area = cv2.contourArea(cnt)
        #if area > 100:
            #cv2.drawContours(roi, [cnt], -1, (0, 255, 0), 2)
    key = cv2.waitKey(30)
    if key == 27:
        break
cap.release()
cv2.destroyAllWindows()