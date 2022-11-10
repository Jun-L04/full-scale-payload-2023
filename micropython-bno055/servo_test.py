# init servo

import utime
from machine import PWM,Pin
servo = PWM(Pin(19))
servo.freq(50)

#define stop,CCW and CW timing in ns
servoStop= 1500000
servoCCW = 1000000
servoCW = 2000000
print("CCW")
servo.duty_ns(servoCCW)
utime.sleep(2)
print("stop")
servo.duty_ns(servoStop)
utime.sleep(1)
print("CW")
servo.duty_ns(servoCW)
utime.sleep(2)
print("stop")
servo.duty_ns(servoStop)

print("end")