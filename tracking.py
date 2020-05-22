from picamera.array import PiRGBArray
from picamera import PiCamera
from detection import Detection
from pid import PID
import cv2
import time
from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

def controls(pid_object, obj_center, frame_center, error_bound):

    # calculate error and perform pid
    error = obj_center - frame_center
    servo_angle = pid_object.update(error, error_bound)

    return servo_angle

if __name__ == "__main__":
    frame_center = (320,240)

    # Servo range of motion in degrees
    pan_range = (10,170) # 0 is left, 180 is right
    tilt_range = (0,60) # 0 is up, 60 is down
    error_bound = (40,40) #

    # Initialize camera object
    camera = PiCamera()
    camera.hflip = True # Mirror the image
    camera.resolution = (640,480)
    raw_capture = PiRGBArray(camera, size=(640,480))

    # allow camera to warm up
    time.sleep(2)

    # Initalize Detection class
    detection = Detection()

    # Initialize PID class
    pid_pan = PID(pan_range, 0.17, 0.105, 0.004)
    pid_pan.initialize()
    pid_tilt = PID(tilt_range, 0.17, 0.12 , 0.004)
    pid_tilt.initialize()

    # Initialize I2C bus
    i2c = busio.I2C(SCL, SDA)
    hat = PCA9685(i2c)
    hat.frequency = 50

    # Define servo objects
    pan = kit.servo[0]
    pan.angle = 90
    tilt = kit.servo[1]
    tilt.angle = 0

    for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
        # perform ball detection
        image = frame.array
        image, obj_center = detection.update(image, frame_center)
        (obj_x, obj_y) = obj_center
        cv2.imshow('Detected Ball', image)

        # PID
        pan_angle = controls(pid_pan, obj_x, frame_center[0], error_bound[0])
        tilt_angle = controls(pid_tilt, obj_y, frame_center[1], error_bound[1])

        # Set servos
        pan.angle = pan_angle
        tilt.angle = tilt_angle

        key = cv2.waitKey(1) & 0xFF
        raw_capture.truncate(0)
        if key == 27:
            break

    # Cleaning up
    camera.close()
    cv2.destroyAllWindows()
    hat.deinit()
    print("exiting...")