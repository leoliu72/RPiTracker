import time

class PID():
    def __init__(self, servo_range, kp=0.25, ki=0, kd=0):
        self.servo_range = servo_range
        self.kp = kp
        self.ki = ki
        self.kd = kd

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
        # Define camera edges. Assumes x range = y range. Fix later
        ERROR_LOWER_BOUND = -70 # arbitrary units
        ERROR_UPPER_BOUND = 70
        error_center = ERROR_LOWER_BOUND + abs(ERROR_UPPER_BOUND - ERROR_LOWER_BOUND) / 2

        # Defines servo angle upper and lower bound ranges
        servo_lower_bound, servo_upper_bound = self.servo_range
        servo_center = (servo_upper_bound - servo_lower_bound) / 2

        # Get servo angle
        pct = abs(servo_angle - error_center) / (ERROR_UPPER_BOUND - error_center)
        if servo_angle >= error_center:
            return int(pct * (servo_upper_bound - servo_center) + servo_center)
        else:
            return int(servo_center - pct * (servo_center - servo_lower_bound))

    # Updates the PID loop
    def update(self, error, sleep = 0.1):
        # Pseudo-loop time
        time.sleep(sleep)

        # Find change in time
        self.curr_time = time.time()
        dt = self.curr_time - self.prev_time

        # Set proportional term to error input
        self.error = error
        # Set integral term to keep adding. Implement antiwindup later
        self.error_int += error * dt
        # Set derivative term
        self.error_der = (error - self.prev_error) / dt

        # Set prev terms for next loop
        self.prev_error = self.error
        self.prev_time = self.curr_time

        servo_angle = int(self.kp * self.error + self.ki * self.error_int + self.kd * self.error_der)

        return servo_angle