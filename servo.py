#Subscale Servo test
#Written by Oori Schubert
#Language: micropython
#Makes servo rotate between 0 and 180 degrees


from machine import Pin,PWM
from time import sleep

pwm = PWM(Pin(17))
pwm.freq(50)

#position 1000 = 0 degrees and position 9000 = 180 degrees
while True:
    for position in range(1000,9000,50): #0 to 180
        pwm.duty_u16(position)
        sleep(0.01)
    for position in range(9000,1000,-50): #180 to 0
        pwm.duty_u16(position)
        sleep(0.01)
