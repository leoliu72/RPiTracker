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
        self.prev_pos = (0,0)

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
            # Find contours of area greater than a thresholds
            cnts = [x for x in contours if cv2.contourArea(x) > 2000]

            # Sort contours by area. Truncate list down to 3 largest contours
            cnts.sort(key = cv2.contourArea, reverse = True)
            if len(cnts) > 3:
                cnts = cnts[0:2]

            # Find moments and centers of the contours
            moments = [cv2.moments(x) for x in cnts]
            cx = [int(moments[i]['m10'] / moments[i]['m00']) for i in range(len(moments))]
            cy = [int(moments[i]['m01'] / moments[i]['m00']) for i in range(len(moments))]

            # Find the contour that is the closest to the previous detected object, call that the new detected object
            distance = [(cx[i] - self.prev_pos[0]) ** 2 + (cy[i] - self.prev_pos[1]) ** 2 for i in range(len(moments))]
            ind = distance.index(min(distance))
            ball_center = (cx[ind], cy[ind])

            # Sets previous position
            self.prev_pos = ball_center
            # print("Camera Loop time: ", time.time() - self.curr_time)

            # return (x, y) center coordinates of the ball
            return(image, ball_center)
        else:
            # If no ball is found, return previous position
            print('No ball detected')
#             print("Camera Loop time: ", time.time() - self.curr_time)
            return (image, self.prev_pos)