#Servo BNO stabilization control 
#Written by Anton Voronov (ravenspired) for the UMass Rocket Team
#Code for subscale rocket flight (Testing the successful transmission of data during flight of the rocket.
#Sends a transmission including made up accelerometer data and a timestamp of seconds since launch.
#Currently tested on an Adafruit Feather Huzzah (4MB Flash) running Micropython 1.19.1, connected to a sparkfun XBee Pro 538 Breakout Board.
#This script sends a transmission every second with the amount of seconds since program was run and with made up accelerometer data.

#Pinouts Pico => Stuff

#BNO:
#17 => SDA
#16 => SCL
#3.3V => VIN
#GND => GND

#Servo:
#ext power supply 5v => 5V
#ext power supply GND => GND (Note that this needs to be a common ground with the pico
#19 => PWM in




import machine
import time
from bno055 import *

from machine import Pin, PWM

servo = PWM(Pin(19))
servo.freq(50) #Set PWM frequency

#define stop,CCW and CW timing in ns

#Our servo has a max ns of 2000000, min ns 1000000, and stops at ns 1500000
servoStop= 1500000
servoCCW = 1450000
servoCW = 1550000

i2c = machine.I2C(0, sda=machine.Pin(16), scl=machine.Pin(17))  # EIO error almost immediately


imu = BNO055(i2c)
calibrated = False


def is_not_accelerating(): #Checks if the rocket is accelerating (if it is falling or flying through the sky)
    if (9 < imu.accel()[2] < 10):
        #print("we are good in acceleration")

        return True
    else:
        #print("we are not good in acceleration")
        return False


def is_not_vertical(): #Checks if rocket is level since servo freaks out otherwise    
    if (-10 < imu.euler()[1] < 10):
        #print("we are good in vertical axis")

        return True
    else:
        #print("we are not good in vertical axis")
        
        return False


def adjust_servo_angle():
    if(is_not_accelerating() & is_not_vertical()): #check that rocket is not moving and level
        #print(imu.euler()[2])
        if imu.euler()[2] > 3: #Leveling code
            #print("spinning CCW")
            servo.duty_ns(servoCCW)
        elif imu.euler()[2] < -3:
            #print("spinning CW")
            servo.duty_ns(servoCW)
        else:
            servo.duty_ns(servoStop) #Tell servo to stop if the BNO is level
    
        
print("program running")
servo.duty_ns(servoStop) #Servo is turned off initially


while True: #Main program

    #print("new loop")

    adjust_servo_angle()
    
    

