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

def set_servo(address, data):
    bus.write_i2c_block_data(address, 0, data) # 0 = start bit
    return -1

if __name__ == "__main__":
    # Initialize I2C bus
    i2c_ch = 1
    bus = smbus.SMBus(i2c_ch)
    address = 0x04

    # Initalize camera class
    camera = PiCamera()
    camera.hflip = True # Mirror the image
    camera.resolution = (640,480)
    camera.framerate = 32
    raw_capture = PiRGBArray(camera, size=(640,480))

    # allow camera to warm up
    time.sleep(2)

    # Initalize Detection class
    detection = Detection()
    frame_center = (320, 240)

    # Servo range of motion in degrees
    pan_range = (20, 100) # 20 is left, 100 is right
    tilt_range = (0,60) # 0 is up, 60 is down

    # Initlialize PID class for pan and tilt servos
    pan = PID(pan_range)
    pan.initialize()
    tilt = PID(tilt_range)
    tilt.initialize()


    try:
        for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
            image, ball_center, enclosing_circle_center, enclosing_circle_radius = detection.update(frame, frame_center)
            if enclosing_circle_center is not None:
                cv2.circle(image, enclosing_circle_center, enclosing_circle_radius, (0,0,255), 2) # drawframe_center circle around object
                cv2.circle(image, ball_center, 5, (255,0,0), 2) # center dot
            cv2.imshow('Detected Ball', image)

            pan_error = ball_center[0] - frame_center[0]
            pan_angle = pan.update(pan_error)
            pan_angle = pan.normalize_servo_angle(pan_angle)

            tilt_error = ball_center[1] - frame_center[1]
            tilt_angle = tilt.update(tilt_error)
            tilt_angle = tilt.normalize_servo_angle(tilt_angle)

            # Send servo angles via I2C to Arduino
            set_servo(address, [pan_angle, tilt_angle])

            key = cv2.waitKey(1) & 0xFF
            raw_capture.truncate(0)

            if key == 27: # Escape key to exit
                break
    finally:
        camera.close()
        print('Closing camera')
        cv2.destroyAllWindows()
        print('Closing script')