import time

class PID():
    def __init__(self, kP=1, kI=0, kD=0):
        self.kP = kP
        self.kI = kI
        self.kD = kD

    def initialize(self):
        # initialize current and previous time
        self.currTime = time.time()
        self.prevTime = self.currTime

        # initialize PID summation terms
        self.prevError = 0
        self.error = 0
        self.errorInt = 0
        self.errorDer = 0

    # Updates the PID loop
    def update(self, error, sleep = 0.2):
        # Sleep for a bit
        time.sleep(sleep)

        # Find change in time
        self.currTime = time.time()
        dt = self.currTime - self.prevTime

        # Set proportional term to error input
        self.error = error
        # Set integral term to keep adding. Implement antiwindup later
        self.errorInt += error * dt
        # Set derivative term
        self.errorDer = (error - self.prevError) / dt

        # Set prev terms for next loop
        self.prevError = self.error
        self.prevTime = self.currTime

        servoAngle = self.kP * self.error + self.kI * self.errorInt + self.kD * self.errorDer

        return servoAngle