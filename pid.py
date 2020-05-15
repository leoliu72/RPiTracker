import time
from collections import deque
import numpy as np

class PID():
    def __init__(self, servo_range, kp=0.25, ki=0, kd=0):
        self.servo_range = servo_range
        self.kp = kp
        self.ki = ki
        self.kd = kd
        # Array for moving avg filter on derivative terms
        self.der_array = deque([],5)

    # Initialize various useful variables
    def initialize(self):
        # initialize current and previous time
        self.curr_time = time.time()
        self.prev_time = self.curr_time

        # initialize PID summation terms
        self.prev_error = 0
        self.error = 0
        self.error_int = 0
        self.error_der = 0

    def normalize_servo_angle(self, servo_angle):
        # Defines servo angle upper and lower bound ranges
        servo_lower_bound, servo_upper_bound = self.servo_range
        servo_center = (servo_upper_bound - servo_lower_bound) / 2

        # Get servo angle
        pct = abs(servo_angle - servo_center) / (servo_upper_bound - servo_center)
        if servo_angle >= servo_center:
            servo_angle = int(pct * (servo_upper_bound - servo_center) + servo_center)
            if servo_angle > servo_upper_bound:
                servo_angle = servo_upper_bound
        else:
            servo_angle = int(servo_center - pct * (servo_center - servo_lower_bound))
            if servo_angle < servo_lower_bound:
                servo_angle = servo_lower_bound

        return servo_angle

    # Updates the PID loop
    def update(self, error, sleep = 0.1):
        time.sleep(sleep)

        self.curr_time = time.time()
        dt = self.curr_time - self.prev_time
        # print("PID loop time: ", dt)

        # Proportional
        self.error = error

        # Integral. Implement antiwindup later
        self.error_int += error * dt

        # Derivative
        # 5 pt moving avg to smooth error term
        self.der_array.append((error - self.prev_error)/dt)
        self.error_der = np.mean(self.der_array)
        # print(self.error_der)

        # Set prev terms for next loop
        self.prev_error = self.error
        self.prev_time = self.curr_time

        # Calculate PID values and servo angle
        p = self.kp * self.error
        i = self.ki * self.error_int
        d = self.kd * self.error_der
        servo_angle = int(p + i + d)
        servo_angle = self.normalize_servo_angle(servo_angle)
        # print('servo angle: ', servo_angle)
        # print('P: ', p, 'I: ', i, ' D: ', d)

        return servo_angle