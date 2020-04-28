from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import smbus

i2c_ch = 1
bus = smbus.SMBus(i2c_ch)

address = 0x04

def writeData(data):
    bus.write_i2c_block_data(address, 0, data)
    return -1

camera = PiCamera()
camera.hflip = True # Mirror the image
camera.resolution = (640,480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640,480))

# allow camera to warm up
time.sleep(2)

class Tracker():
    def __init__(self):
        
        # Initialize variables for PID control and CV
        self.panPrevError = 0
        self.tiltPrevError = 0
        
        # HSV thresholds
        
        self.lowerGreen = (29,86,6)
        self.upperGreen = (64,255,255)
        
        print('Finished init')
    
    def vision(self):
        try:
            while True:
                for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
                    image = frame.array # read image
                    
                    blur = cv2.GaussianBlur(image.copy(), (7,7), 0)
                ##    blur = cv2.medianBlur(blur, 7)
                    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
                    mask = cv2.inRange(hsv, self.lowerGreen, self.upperGreen)
                ##    cv2.imshow('Before morpho', mask)
                    
                ##    # More filtering at the cost of speed. Removes more noise, but just HSV masking
                ##    # seems to be sufficient for now
                ##    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
                ##    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
                ##    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                ####    cv2.imshow('After morpho', mask)
            
                    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                    if len(contours) > 0 and max([cv2.contourArea(x) for x in contours]) > 100:
                        c = max(contours, key = cv2.contourArea)
                        M = cv2.moments(c) # find moments
                        
                        cx = int(M['m10']/M['m00']) # x coordinate of center
                        cy = int(M['m01']/M['m00']) # y coordinate of center
                        ((x,y), radius) = cv2.minEnclosingCircle(c)
                        
                        cv2.circle(image, (int(x),int(y)), int(radius), (0,0,255), 2) # draw circle around object
                        cv2.circle(image, (cx, cy), 5, (255,0,0), 2) # draw center dot
                        cv2.imshow('Detected Ball', image)
                        
                        kp = 0.02
                        pan = cx
                        tilt = cy
                        
                        panError = pan - self.panPrevError
                        print('panError: ', self.panPrevError)

                        # self.panPrevError = panError
                        panAngle = int(kp * panError)
                        if panAngle >= 100:
                            panAngle = 100
                        elif panAngle < 20:
                            panAngle = 20
                        # print('pan angle: ', panAngle)
                        
                        tiltError = tilt - self.tiltPrevError
                        # print('tiltError', tiltError)
                        self.tiltPrevError = tiltError
                        tiltAngle = int(kp * panError)
                        if tiltAngle >= 60:
                            tiltAngle = 60
                        elif tiltAngle <= 0:
                            tiltAngle = 0
                        # print('tile angle: ', tiltAngle)
            
    ##        writeData([panAngle, tiltAngle])
                    else:
                        print('No ball detected')
                    
                    key = cv2.waitKey(1) & 0xFF
                    rawCapture.truncate(0)

                    if key == 27: # Escape key to exit
                        break
        finally:
            camera.close()
            cv2.destroyAllWindows()
            print('Closing')

if __name__ == '__main__':
    tracker = Tracker()