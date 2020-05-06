from picamera.array import PiRGBArray
from picamera import PiCamera
from multiprocessing import Manager
from multiprocessing import Process
from detection import Detection
from pid import PID
import time
import cv2
import smbus
import sys
import signal

# A function to kill processes
def e_stop(sig, frame):
    print('User requested exit. Exiting...')

    sys.exit()

# Process for sending servo angles to Arduino
def set_servo(pan_angle, tilt_angle):
    # Handles keyboard interrupts to exit script
    signal.signal(signal.SIGINT, e_stop)

    # Initialize I2C bus
    i2c_ch = 1
    bus = smbus.SMBus(i2c_ch)
    address = 0x04

    # Send servo angles to Arduino
    while True:
        try:
            time.sleep(0.1)
            bus.write_i2c_block_data(address, 0, [pan_angle.value, tilt_angle.value]) # 0 = start bit
        except OSError as e:
            print(e)

def image_processing(frame_center, obj_x, obj_y):
    # Handles keyboard interrupts to exit script
    signal.signal(signal.SIGINT, e_stop)

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

    for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
        # perform ball detection
        image, obj_center, enclosing_circle_center, enclosing_circle_radius = detection.update(frame, frame_center)
        (obj_x.value, obj_y.value) = obj_center

        # CV results
        if enclosing_circle_center is not None:
            cv2.circle(image, enclosing_circle_center, enclosing_circle_radius, (0,0,255), 2) # drawframe_center circle around object
            cv2.circle(image, obj_center, 5, (255,0,0), 2) # center dot
        cv2.imshow('Detected Ball', image)

        key = cv2.waitKey(1) & 0xFF
        raw_capture.truncate(0)

def controls(output, servo_range, p, i ,d, center_obj, center_frame):
    # Handles keyboard interrupts to exit script
    signal.signal(signal.SIGINT, e_stop)

    obj = PID(servo_range, p.value, i.value, d.value)
    obj.initialize()

    # Continuously run control loop
    while True:
        # Calculate error
        error = center_obj.value - center_frame.value

        # Calculate servo output using error
        output.value = obj.update(error)

if __name__ == "__main__":
    frame_center = (320, 240)

    # Servo range of motion in degrees
    pan_range = (0, 180) # 0 is left, 180 is right
    tilt_range = (0,60) # 0 is up, 60 is down

    with Manager() as manager:
        # (x,y) of the frame center
        frame_x = manager.Value("i", frame_center[0])
        frame_y = manager.Value("i", frame_center[1])

        # (x,y) of the object center
        obj_x = manager.Value("i", 0)
        obj_y = manager.Value("i", 0)

        # servo angles to be sent to Arduino
        pan_angle = manager.Value("i", 0)
        tilt_angle = manager.Value("i", 0)

        # PID constants
        pan_p = manager.Value("f", 0.085)
        pan_i = manager.Value("f", 0.085)
        pan_d = manager.Value("f", 0.00005)

        tilt_p = manager.Value("f", 0.07)
        tilt_i = manager.Value("f", 0.02)
        tilt_d = manager.Value("f", 0)

        process_detection = Process(target=image_processing, args=(frame_center, obj_x, obj_y))
        process_panning = Process(target=controls, args=(pan_angle, pan_range, pan_p, pan_i , pan_d, obj_x, frame_x))
        process_tilting = Process(target=controls, args=(tilt_angle, tilt_range, tilt_p, tilt_i, tilt_d, obj_y, frame_y))
        process_set_servo = Process(target=set_servo, args=(pan_angle, tilt_angle))

        process_detection.start()

        #let camera warm up before starting PID
        time.sleep(3)

        process_panning.start()
        # process_tilting.start()
        process_set_servo.start()

        process_detection.join()
        process_panning.join()
        # process_tilting.join()
        process_set_servo.join()

        cv2.destroyAllWindows()
        print('Closing script')