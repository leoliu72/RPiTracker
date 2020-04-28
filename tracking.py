from picamera.array import PiRGBArray
from picamera import PiCamera
from multiprocessing import Manager
from multiprocessing import Process
from detection import Detection
from pid import PID
import time
import cv2
import smbus
print('All libraries imported')

if __name__ == "__main__":
    camera = PiCamera()
    camera.hflip = True # Mirror the image
    camera.resolution = (640,480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640,480))

    # allow camera to warm up
    time.sleep(2)

    # Center of the frame for when no ball is detected
    frameCenter = (320,240)
    detection = Detection()

    try:
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            image, ball_center, enclosing_circle_center, enclosing_circle_radius = detection.update(frame, frameCenter)
            cv2.circle(image, enclosing_circle_center, enclosing_circle_radius, (0,0,255), 2) # draw circle around object
            cv2.circle(image, ball_center, 5, (255,0,0), 2) # dcenter dot
            cv2.imshow('Detected Ball', image)

            key = cv2.waitKey(1) & 0xFF
            rawCapture.truncate(0)

            if key == 27: # Escape key to exit
                break
    finally:
        camera.close()
        print('Closing camera')
        cv2.destroyAllWindows()
        print('Closing script')