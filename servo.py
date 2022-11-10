#Subscale Servo calibration
#Written by Oori Schubert
#Written in micropython
#Sets BNO055 to 0 degrees using servo


from machine import Pin,PWM,I2C
from time import sleep
from bno055 import *

pwm = PWM(Pin(15))
pwm.freq(50)

i2c = I2C(0, sda=Pin(16), scl=Pin(17)) #
imu = BNO055(i2c)

#set servo to 0 degrees based on x axis
def setServo() -> float: #is it a float or int?
    calibrated = False
    pwm.duty_u16(1000) #set servo to 0
    while True:
        if imu.calibrated():
            calibrated = True
            break
        elif not calibrated:
                calibrated = imu.calibrated()
                print('Calibration required: sys {} gyro {} accel {} mag {}'.format(*imu.cal_status()))
                sleep(1)
    #read x axis
    x = imu.gyro()[0]
    print(x)

    #convert to duty cycle
    duty = int((x/180)*65535) #not correct?: need to convert to 0-180 degrees
    print(duty)

    #set servo to x degrees
    pwm.duty_u16(-duty) #need to set to negative in order to get true zero
    sleep(0.1)
    return imu.gyro()[0] #returns new x axis for verification

setServo()
