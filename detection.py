import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import numpy as np
import time

class Detection():
    def __init__(self):
        # Define HSV thresholds for detection
        self.lower_green = (29,86,6)
        self.upper_green = (64,255,255)
        self.curr_time = time.time()
        self.prev_time = self.curr_time

    def update(self, frame, frame_center):
        self.curr_time = time.time()
        # read image
        image = frame.array

        blur = cv2.GaussianBlur(image.copy(), (11,11), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        # cv2.imshow("After dilation", mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0 and max([cv2.contourArea(x) for x in contours]) > 2000:
            # Assume largest contour is the ball
            c = max(contours, key = cv2.contourArea)

            # Find ball_center to send to PID
            M = cv2.moments(c) # find moments
            cx = int(M['m10']/M['m00']) # x coordinate of center
            cy = int(M['m01']/M['m00']) # y coordinate of center
            ball_center = (cx, cy)

            # Bound the ball for visualization
            ((x,y), r) = cv2.minEnclosingCircle(c)
            enclosing_circle_center = (int(x), int(y))
            enclosing_circle_radius = int(r)
#             print("Camera Loop time: ", time.time() - self.curr_time)
            # return (x, y) center coordinates of the ball
            return(image, ball_center, enclosing_circle_center, enclosing_circle_radius)
        else:
            # If no ball is found, return center of the frame -> error goes to zero -> servo stops moving
            print('No ball detected')
#             print("Camera Loop time: ", time.time() - self.curr_time)
            return (image, frame_center, None, None)