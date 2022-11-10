



import machine
import time
from bno055 import *

from machine import Pin, PWM

servo = PWM(Pin(19))
servo.freq(50) #Set PWM frequency

#define stop,CCW and CW timing in ns

#Our servo has a max ns of 2000000, min ns 1000000, and stops at ns 1500000
servoStop= 1500000
servoCCW = 1400000
servoCW = 1600000

i2c = machine.I2C(0, sda=machine.Pin(16), scl=machine.Pin(17))  # EIO error almost immediately


imu = BNO055(i2c)
calibrated = False


def is_not_accelerating(): #Checks if the rocket is accelerating (if it is falling or flying through the sky)
    return (9 < imu.accel()[2] < 10)


def is_not_vertical(): #Checks if rocket is level since servo freaks out otherwise    
    return (-10 < imu.euler()[1] < 10):


def adjust_servo_angle():
    if(is_not_accelerating() & is_not_vertical()): #check that rocket is not moving and level
        print(imu.euler()[2])
        if imu.euler()[2] > 3: #Leveling code
            print("spinning CCW")
            servo.duty_ns(servoCCW)
        elif imu.euler()[2] < -3:
            print("spinning CW")
            servo.duty_ns(servoCW)
        else:
            servo.duty_ns(servoStop) #Tell servo to stop if the BNO is level
    
        

servo.duty_ns(servoStop) #Servo is turned off initially


while True: #Main program 
    time.sleep(0.01)
    adjust_servo_angle()
    
    

