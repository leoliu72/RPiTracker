import time
from collections import deque
import numpy as np

class PID():
    def __init__(self, servo_range, kp, ki, kd):
        self.servo_range = servo_range
        self.kp = kp
        self.ki = ki
        self.kd = kd
        # Array for moving avg filter on error bound
        self.error_array = deque([],5)

    # Initialize various useful variables
    def initialize(self):
        # initialize current and previous time
        self.curr_time = time.time()
        self.prev_time = self.curr_time

        # initialize PID summation terms
        self.prev_error = 0
        self.error_int = 0
        self.prev_servo_angle = 0


    def normalize_servo_angle(self, servo_angle):
        # Defines servo angle upper and lower bound ranges
        servo_lower_bound, servo_upper_bound = self.servo_range
        servo_center = (servo_upper_bound - servo_lower_bound) / 2

        # Get servo angle
        pct = abs(servo_angle - servo_center) / (servo_upper_bound - servo_center)
        if servo_angle >= servo_center:
            servo_angle = int(np.around(pct * (servo_upper_bound - servo_center) + servo_center))
            if servo_angle > servo_upper_bound:
                servo_angle = servo_upper_bound
        else:
            servo_angle = int(np.around(servo_center - pct * (servo_center - servo_lower_bound)))
            if servo_angle < servo_lower_bound:
                servo_angle = servo_lower_bound

        return servo_angle

    def speed_limit(self, servo_angle):
        """ Sets the slew rate of servos to account for mechanical constraints"""
        limit = 6
        if abs(servo_angle - self.prev_servo_angle) >= limit:
            if servo_angle > self.prev_servo_angle:
                servo_angle = self.prev_servo_angle + limit
            else:
                servo_angle = self.prev_servo_angle - limit

        return servo_angle

    # Updates the PID loop
    def update(self, error, error_bound):
        self.curr_time = time.time()
        dt = self.curr_time - self.prev_time

        # Integral. Implement antiwindup later
        self.error_int += error * dt

        # Derivative
        error_der = (error - self.prev_error) / dt

        # Calculate PID values and servo angle
        if abs(np.mean(self.error_array)) <= error_bound:
            # If the error is within some distance of the center of the frame, don't move
            servo_angle = self.prev_servo_angle
        else:
            # Otherwise, do PID
            p = self.kp * error
            i = self.ki * self.error_int
            d = self.kd * error_der
            servo_angle = p + i + d
            servo_angle = self.normalize_servo_angle(servo_angle)
            servo_angle = self.speed_limit(servo_angle)

         # Set prev terms for next loop
        self.prev_servo_angle = servo_angle
        self.error_array.append(error)
        self.prev_error = error
        self.prev_time = self.curr_time

        return servo_angle