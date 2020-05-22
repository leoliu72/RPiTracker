import cv2
import numpy as np

class Detection():
    def __init__(self):
        # Define HSV thresholds for detection
        self.lower_green = (29,86,6)
        self.upper_green = (64,255,255)
        self.prev_center = (320,240)

    def update(self, image, frame_center):
        # Filter tennis ball by color
        blur = cv2.GaussianBlur(image.copy(), (11,11), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Detect tennis ball using circle detection
        circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT,1,minDist=400,
            param1=15,param2=10,minRadius=15,maxRadius=100)

        if circles is not None:
            # find center and radius of circles
            centers = [(int(i[0]),int(i[1])) for i in circles[0,:]]
            radii = [int(i[2]) for i in circles[0,:]]

            # calculate distance from previous center for each circle. Closest circle is the ball
            distance = [(centers[i][0] - self.prev_center[0]) ** 2 + (centers[i][1] - self.prev_center[1]) ** 2 for i in range(len(circles))]
            ind = distance.index(min(distance))
            ball_center = (centers[ind])

            # draw ball boundaries
            cv2.circle(image, ball_center, radii[ind], (255,0,0), 3)
            cv2.circle(image, ball_center, 2, (0,0,255), 3) # center dot

            # Sets previous position
            self.prev_center = ball_center

            # return (x, y) center coordinates of the ball
            return(image, ball_center)
        else:
            # If no ball is found, return previous position
            print('No ball detected')
            return (image, self.prev_center)