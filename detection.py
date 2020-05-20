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

            # Sort contours by area
            cnts.sort(key = cv2.contourArea, reverse = True)

            # Truncate list down to 3 largest contours
            if len(cnts) > 3:
                cnts = cnts[0:2]

            # Find moments and centers of the contours
            moments = [cv2.moments(x) for x in cnts]
            cx = [0] * len(moments)
            cy = [0] * len(moments)
            distance = [0] * len(moments)
            for i in range(len(moments)):
                cx[i] = int(moments[i]['m10'] / moments[i]['m00'])
                cy[i] = int(moments[i]['m01'] / moments[i]['m00'])
                distance[i] = (cx[i] - self.prev_pos[0]) ** 2 + (cy[i] - self.prev_pos[1]) ** 2

            ind = distance.index(min(distance))
            x = cx[ind]
            y = cy[ind]
            ball_center = (x,y)
            r = 13
            cv2.circle(image,ball_center, r, (0,255,0), 3)
            cv2.imshow("Ball", image)

            #cv2.drawContours(image, cnts, -1, (0,255,0), 3)
            #cv2.imshow("contours", image)

            # Bound the ball for visualization
            # ((x,y), r) = cv2.minEnclosingCircle(c)
            enclosing_circle_center = (int(x), int(y))
            enclosing_circle_radius = int(r)

            # Sets previous position in case ball is not found the next loop
            self.prev_pos = ball_center
#             print("Camera Loop time: ", time.time() - self.curr_time)

            # return (x, y) center coordinates of the ball
            return(image, ball_center, enclosing_circle_center, enclosing_circle_radius)
        else:
            # If no ball is found, return previous position
            print('No ball detected')
#             print("Camera Loop time: ", time.time() - self.curr_time)
            return (image, self.prev_pos, None, None)

if __name__ == "__main__":
    # Initialize camera object
    camera = PiCamera()
    camera.hflip = True # Mirror the image
    camera.resolution = (640,480)
    raw_capture = PiRGBArray(camera, size=(640,480))

    frame_center = (320, 240)

    # allow camera to warm up
    time.sleep(2)

    # Initalize Detection class
    detection = Detection()

    for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
        # perform ball detection
        image, obj_center, enclosing_circle_center, enclosing_circle_radius = detection.update(frame, frame_center)

        # CV results
        # if enclosing_circle_center is not None:
#             cv2.circle(image, enclosing_circle_center, enclosing_circle_radius, (0,0,255), 2) # drawframe_center circle around object
#             cv2.circle(image, obj_center, 5, (255,0,0), 1) # center dot
#         cv2.imshow('Detected Ball', image)


        key = cv2.waitKey(1) & 0xFF
        raw_capture.truncate(0)

        if key == 27:
            camera.close()
            cv2.destroyAllWindows()