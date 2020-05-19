from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)
import time
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL, SDA)
hat = PCA9685(i2c)
hat.frequency = 50
servo_sleep = 0.025
servo_step = 5

for i in range(0,5):
    for angle in range(40,140,servo_step):
        kit.servo[0].angle = angle
        time.sleep(servo_sleep)
    for angle in range(140,40,-servo_step):
        kit.servo[0].angle = angle
        time.sleep(servo_sleep)
    for angle in range(0,60,servo_step):
        kit.servo[1].angle = angle
        time.sleep(servo_sleep)
    for angle in range(60,0,-servo_step):
        kit.servo[1].angle = angle
        time.sleep(servo_sleep)
hat.deinit()