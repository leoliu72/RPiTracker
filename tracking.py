from picamera.array import PiRGBArray
from picamera import PiCamera
from multiprocessing import Manager
from multiprocessing import Process
from detection import Detection
from pid import PID
import time
import cv2
import sys
import signal
from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

# Initialize I2C bus
i2c = busio.I2C(SCL, SDA)
hat = PCA9685(i2c)
hat.frequency = 50

# A function to kill processes
def e_stop(sig, frame):
    print('User requested exit.')

    # Close servo and camera processes
    hat.deinit()
    # camera.close()
    cv2.destroyAllWindows()
    print('All processes shut down. Exiting...')

    sys.exit()

# Process for sending servo angles to Arduino
def set_servo(pan_angle, tilt_angle):
    # Handles keyboard interrupts to exit script
    signal.signal(signal.SIGINT, e_stop)

    # Define servo objects
    pan = kit.servo[0]
    tilt = kit.servo[1]

    while True:
        try:
            pan.angle = pan_angle.value
            tilt.angle = tilt_angle.value
            # print('Pan: ', pan_angle.value)
            # print('Tilt: ', tilt_angle.value)
        except (OSError, TypeError) as e:
            print(e)

def image_processing(frame_center, obj_x, obj_y):
    # Handles keyboard interrupts to exit script
    signal.signal(signal.SIGINT, e_stop)

    # Initialize camera object
    camera = PiCamera()
    camera.hflip = True # Mirror the image
    camera.resolution = (640,480)
    raw_capture = PiRGBArray(camera, size=(640,480))

    # allow camera to warm up
    time.sleep(2)

    # Initalize Detection class
    detection = Detection()

    for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
        # perform ball detection
        image, obj_center = detection.update(frame, frame_center)
        (obj_x.value, obj_y.value) = obj_center

        # CV results
        if obj_center is not None:
            cv2.circle(image, obj_center, 3, (0,0,255), 3) # center dot
            cv2.rectangle(image, (260,180), (380,300), (255,0,0),1)
        cv2.imshow('Detected Ball', image)

        key = cv2.waitKey(1) & 0xFF
        raw_capture.truncate(0)

def controls(output, servo_range, p, i ,d, obj_center, frame_center):
    # Handles keyboard interrupts to exit script
    signal.signal(signal.SIGINT, e_stop)

    pid = PID(servo_range, p.value, i.value, d.value)
    pid.initialize()

    # Continuously run control loop
    while True:
        # Calculate error
        error = obj_center.value - frame_center

        # Calculate servo output using error
        output.value = pid.update(error)

if __name__ == "__main__":
    frame_center = (320,240)

    # Servo range of motion in degrees
    pan_range = (10,170) # 0 is left, 180 is right
    tilt_range = (0,60) # 0 is up, 60 is down

    with Manager() as manager:
        # (x,y) of the object center
        obj_x = manager.Value("i", 0)
        obj_y = manager.Value("i", 0)

        # servo angles
        pan_angle = manager.Value("i", 0)
        tilt_angle = manager.Value("i", 0)

        # PID constants
        pan_p = manager.Value("f", 0.105)
        pan_i = manager.Value("f", 0.095)
        pan_d = manager.Value("f", 0.004)

        tilt_p = manager.Value("f", 0.095)
        tilt_i = manager.Value("f", 0.083)
        tilt_d = manager.Value("f", 0.002)

        process_detection = Process(target=image_processing, args=(frame_center, obj_x, obj_y))
        process_panning = Process(target=controls, args=(pan_angle, pan_range, pan_p, pan_i , pan_d, obj_x, frame_center[0]))
        process_tilting = Process(target=controls, args=(tilt_angle, tilt_range, tilt_p, tilt_i, tilt_d, obj_y, frame_center[1]))
        process_set_servo = Process(target=set_servo, args=(pan_angle, tilt_angle))

        process_detection.start()
        #let camera warm up before starting PID
        time.sleep(3)

        process_panning.start()
        process_tilting.start()
        process_set_servo.start()

        process_detection.join()
        process_panning.join()
        process_tilting.join()
        process_set_servo.join()

        camera.close()