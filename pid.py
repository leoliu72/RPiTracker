import time

class PID():
    def __init__(self, servo_range, kp=1, ki=0, kd=0):
        self.servo_range = servo_range
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def initialize(self):
        # initialize current and previous time
        self.curr_time = time.time()
        self.prev_time = self.curr_time

        # initialize PID summation terms
        self.prev_error = 0
        self.error = 0
        self.error_int = 0
        self.error_der = 0

    # Updates the PID loop
    def update(self, error, sleep = 0.2):
        # Sleep for a bit
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

        servo_angle = self.kp * self.error + self.ki * self.error_int + self.kd * self.error_der

        # Bound movement of servo_angle
        if servo_angle < self.servo_range[0]:
            servo_angle = self.servo_range[0]
        elif servo_angle > self.servo_range[1]:
            servo_angle = self.servo_range[1]

        return servo_angle