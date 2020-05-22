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
        self.prev_center = (320,240)

    def update(self, image, frame_center):
        self.curr_time = time.time()

        blur = cv2.GaussianBlur(image.copy(), (11,11), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        # cv2.imshow("After dilation", mask)

        circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT,1,minDist=400,
            param1=15,param2=10,minRadius=15,maxRadius=100)

        if circles is not None:
            # find center and radius of circles
            centers = [(int(i[0]),int(i[1])) for i in circles[0,:]]
            radii = [int(i[2]) for i in circles[0,:]]

            # calculate distance from previous center for each circle. Closest circle is the ball
            distance = [(centers[i][0] - self.prev_center[0]) ** 2 + (centers[i][1] - self.prev_center[1]) ** 2
                for i in range(len(circles))]
            ind = distance.index(min(distance))
            ball_center = (centers[ind])

            # draw center dot
            # cv2.circle(image, ball_center, 3, (0,0,255), 3)
            # cv2.imshow("circles", image)

            # Sets previous position
            self.prev_center = ball_center
            # print("Camera Loop time: ", time.time() - self.curr_time)

            # return (x, y) center coordinates of the ball
            return(image, ball_center)
            return(image, ball_center)
        else:
            # If no ball is found, return previous position
            print('No ball detected')
#             print("Camera Loop time: ", time.time() - self.curr_time)
            return (image, self.prev_center)

# if __name__ == "__main__":
#     frame_center = (320, 240)

#     camera = PiCamera()
#     camera.hflip = True
#     camera.resolution = (640,480)
#     raw_capture = PiRGBArray(camera, size=(640,480))

#     time.sleep(2)
#     detection = Detection()

#     for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
#         image, obj_center = detection.update(frame, frame_center)

#         key = cv2.waitKey(1) & 0xFF
#         raw_capture.truncate(0)

#         if key == 27:
#             camera.close()
#             cv2.destroyAllWindows()