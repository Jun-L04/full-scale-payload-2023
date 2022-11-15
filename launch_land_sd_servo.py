import machine
import uos
import time
from bno055 import *
from bno055_base import BNO055_BASE
from queue import *#do Queue(max_size, threshold) to make object
from math import floor
import sdcard
from sys import exit#only for testing

gc.collect()


uart = machine.UART(0, baud_rates[9600]) # setup uart object (uart0 maps to pin 1 on the pico)
uart.init(9600, parity=None, stop=1) # initialize the serial connection with given parameters
time.sleep(0.5)
uart.write('Initial Transmission - Rocket was connected to power')

#To add transmissions, use uart.write(string)

class ForceLandingException(Exception):
    pass
init_time = time.ticks_ms()
def mean(arr, length):
    sum = 0
    for e in arr:
        if e != None:
            sum += e
    return sum/length
#Input: Queue to find mean/variance
#Output: array: [mean, variance]
def mean_variance(queue):
    size = queue.get_size()
    arr = queue.get_array()
    mean_val = mean(arr, size)
    sum = 0
    for e in arr:
        if e!= None:
            sum += (e-mean_val)**2
    return [mean_val, sum/size]

def calculate_max_size(sample_frequency, queue_interval):
    return floor(sample_frequency * queue_interval / 1000 * 1.25) #calculate maximum size of queue needed by frequency (samples/s) * interval to keep track of (ms) / 1000 (ms/s) * 1.25 room for error

#Inputs: Array of functions that should be run every x ms
#		Array of intervals of time between each run of its corresponding function
#Outputs: None
#Each function in func_arr should raise an exception if the loop should stop being run
def do_every(func_arr, interval_arr):
    prevTimes = [-999999]*len(func_arr)
    while True:
        for i, func in enumerate(func_arr):
            if(time.ticks_diff(time.ticks_ms(), prevTimes[i]) > interval_arr[i]):
                try:
                    func()
                    prevTimes[i] = time.ticks_ms()
                except StopIteration:#Leave the while loop
                    return
check_override_interval = 15000#Check for override every 15 seconds
def check_time_land_override():
    if time.ticks_diff(time.ticks_ms(), init_time) > 3600000:#If time is greater than 1 hr (in ms), then skip to landing
        raise ForceLandingException
# Pico: hard I2C doesn't work without this patch
# https://github.com/micropython/micropython/issues/8167#issuecomment-1013696765

i2c = machine.I2C(0, sda=machine.Pin(16), scl=machine.Pin(17))  # EIO error almost immediately
#init imu
imu = BNO055(i2c)

# Assign chip select (CS) pin (and start it high)
cs = machine.Pin(9, machine.Pin.OUT)
# Intialize SPI peripheral (start with 1 MHz)
spi = machine.SPI(1,
                  baudrate=1000000,
                  polarity=0,
                  phase=0,
                  bits=8,
                  firstbit=machine.SPI.MSB,
                  sck=machine.Pin(10),
                  mosi=machine.Pin(11),
                  miso=machine.Pin(8))

# Initialize SD card
sd = sdcard.SDCard(spi, cs)

# Mount filesystem
vfs = uos.VfsFat(sd)
uos.mount(vfs, "/sd")

imu_data = open("/sd/imu_data.csv" , "w")#Need to close this at the end of the program
imu_data.write("Time, Temperature, Mag X, Mag Y, Mag Z, Gyro X, Gyro Y, Gyro Z, Acc X, Acc Y, Acc Z, Lin Acc X, Lin Acc Y, Lin Acc Z, Gravity X, Gravity Y, Gravity Z, Euler X, Euler Y, Euler Z\n")
flight_log = open("/sd/flight_log.txt", "w")
imu_data.write(str(time.ticks_diff(time.ticks_ms(), init_time)/1000.0) + "," + str(imu.temperature()) + "," + '{:5.3f},{:5.3f},{:5.3f},'.format(*imu.mag()) + '{:5.3f},{:5.3f},{:5.3f},'.format(*imu.gyro()) + '{:5.3f},{:5.3f},{:5.3f},'.format(*imu.accel()) + '{:5.3f},{:5.3f},{:5.3f},'.format(*imu.lin_acc()) + '{:5.3f},{:5.3f},{:5.3f},'.format(*imu.gravity()) + '{:4.3f},{:4.3f},{:4.3f}'.format(*imu.euler()) + "\n")


def write_to_imu_data():
    imu_data.write(str(time.ticks_diff(time.ticks_ms(), init_time)/1000.0) + "," + str(imu.temperature()) + "," + '{:5.3f},{:5.3f},{:5.3f},'.format(*imu.mag()) + '{:5.3f},{:5.3f},{:5.3f},'.format(*imu.gyro()) + '{:5.3f},{:5.3f},{:5.3f},'.format(*imu.accel()) + '{:5.3f},{:5.3f},{:5.3f},'.format(*imu.lin_acc()) + '{:5.3f},{:5.3f},{:5.3f},'.format(*imu.gravity()) + '{:4.3f},{:4.3f},{:4.3f}'.format(*imu.euler()) + "\n")

    #if time.ticks_diff(time.ticks_ms(), init_time) > 10000:
        #raise StopIteration ONLY USED FOR TESTING PURPOSES
imu_data_frequency = 100 #Hz
imu_data_interval = 1/imu_data_frequency*1000 #ms


#Declaring queues. They must be accessible via the global context
time_queue = None
accel_queue = None

#Decarling landing override. Must be accesible via global context
landing_override = False
def check_override():#If payload goes for 1 hr, then override launch/landing detection straight to landed phase
    if time.ticks_diff(time.ticks_ms(), init_time)>3_600_000:#Longer than 1 hour
        global landing_override
        landing_override = True
        raise StopIteration
#Inputs: Interval: How much acceleration data should be kept track of (eg last 2000 ms, interval = 2000)
#						NOTE: This does not mean acceleration is updated every interval ms
#Output: A function that can be passed into do_every to update the time and acceleration queues
def make_data_updater(interval):
    def updater():
        cur_time = time.ticks_ms()
        time_queue.enqueue(cur_time)
        accel_data = imu.accel()
        accel_queue.enqueue((accel_data[0]**2+accel_data[1]**2+accel_data[2]**2)**.5) 
        while time.ticks_diff(cur_time, time_queue.peek()) > interval:
            time_queue.dequeue()
            accel_queue.dequeue()
    return updater

#-----CALIBRATION PHASE-----
def calibration_fn():
    flight_log.write('\nCalibration required: sys {} gyro {} accel {} mag {}'.format(*imu.cal_status()))
    if imu.calibrated():
        flight_log.write("\nCALIBRATED!");
        #bytearray(b'\xfa\xff\x00\x00\xe9\xffF\x04\x13\x01|\xff\xff\xff\x00\x00\x00\x00\xe8\x03\xec\x01')
        raise StopIteration
#Check for calibration every 1000 ms
######------do_every([calibration_fn], [1000])
#The IMU is calibrated at this point
#The reference gravity must be calculated in setup for the rest of the program to use
#For stopping the executition of program during testing
offset_arr = bytearray(b'\xfa\xff\x00\x00\xe9\xffF\x04\x13\x01|\xff\xff\xff\x00\x00\x00\x00\xe8\x03\xec\x01')
imu.set_offsets(offset_arr)

flight_log.write("\nIMU is calibrated")
flight_log.write("\nTime (ms): " + str(time.ticks_ms()))
uart.write("IMU is calibrated")

#-----SETUP PHASE-----
GRAVITY = 9.8
queue_frequency = 5 #Hz
check_interval = 5000#time between variance checks
accel_sample_interval = 1000 / queue_frequency #calculate time between samples in ms based on desired frequency
max_size = calculate_max_size(queue_frequency, check_interval)

#Queue(maximum length of queue, initial threshold)
time_queue = Queue(max_size, None)
accel_queue = Queue(max_size, GRAVITY)
data_updater = make_data_updater(check_interval)
def find_reference_gravity():#CHECK HERE FOR SOMEWHAT ARBITRARY VALUES
    global GRAVITY
    if time.ticks_diff(time.ticks_ms(), time_queue.peek()) < check_interval*0.5:
        return
    stats = mean_variance(accel_queue)
    if stats[1] < 0.1 and stats[0] > 9.5 and stats[0] < 10.1:#ARBITRARY: if variance less than .1 m/s^2, also check that mean is reasonable (g+-.3)
        #May also want to check that queue has big enough number of elements
        #Set reference gravity
        flight_log.write("\nCalculated Gravity: " + str(stats))
        GRAVITY = stats[0]*1.25 #ARBITRARY: Multiply by 1.1 to give some room for error
        raise StopIteration
if not landing_override:
    do_every([data_updater, find_reference_gravity, check_override], [accel_sample_interval, check_interval, check_override_interval])
#GRAVITY is set at this point
flight_log.write("\nGravity has been set: " + str(GRAVITY))
flight_log.write("\nTime (ms): " + str(time_queue.peek()))
uart.write("Gravity has been calculated. Ready for flight.")
#-----BEFORE FLIGHT PHASE-----
#Reset time and acceleration queues
queue_frequency = 50 #Hz
burn_time = 2970 * .75#Multiply by .75 for some room for error
accel_sample_interval = 1000 / queue_frequency #calculate time between samples in ms based on desired frequency
max_size = calculate_max_size(queue_frequency, burn_time)

time_queue = Queue(max_size, None)
accel_queue = Queue(max_size, GRAVITY)

def check_launch():
    if time.ticks_diff(time.ticks_ms(), time_queue.peek()) < 0.5*burn_time:
        return#Dont check for launch if there's not enough data in the queue yet
    if accel_queue.get_proportion_above_threshold() > 0.95:#ARBITRARY: If 95% of the data is above the gravity threshold, then check variance
        if mean_variance(accel_queue)[1] > .3:#ARBITRARY: If the variance is above 0.5 m/s^2, then launch is detected
            raise StopIteration

data_updater = make_data_updater(burn_time)#Function to update accelerations, keeping only last burn_time ms in the queue


check_launch_interval = 1000 #ARBITRARY: check for launch every check_launch_interval ms
'''COMMENTED OUT FOR TESTING PURPOSE ONLY
if not landing_override
    do_every([write_to_imu_data, data_updater, check_launch, check_override], [imu_data_interval, accel_sample_interval, check_launch_interval, check_override_interval])
'''
#The rocket has launched at this point
flight_log.write("\nThe rocket has launched")
flight_log.write("\nTime (ms): " + str(time_queue.peek()))
uart.write("Launch Detected")
#-----IN FLIGHT PHASE-----
#Reset time and acceleration queues
queue_frequency = 25 #Hz
check_interval = 10000 #Want to keep track fo last 10 seconds of acceleration data
accel_sample_interval = 1000 / queue_frequency #calculate time between samples in ms based on desired frequency
max_size = calculate_max_size(queue_frequency, check_interval)

time_queue = Queue(max_size, None)
accel_queue = Queue(max_size, GRAVITY)

def check_landing():
    if time.ticks_diff(time.ticks_ms(), time_queue.peek()) < 0.5*check_interval:
        return#Dont check for landing if there's not enough data in the queue yet
    if accel_queue.get_proportion_above_threshold() < 0.05:#ARBITRARY: If no more than 5% of the acceleration data is above the gravity threshold, then check variance
        if mean_variance(accel_queue)[1] < .25:#ARBITRARY: If variance is below 0.25 m/s^2, then landing is detected
            raise StopIteration
data_updater = make_data_updater(check_interval)
check_landing_interval = 3000#ARBITRARY: Check for landing every 3 seconds
if not landing_override:
    do_every([write_to_imu_data, data_updater, check_landing, check_override], [imu_data_interval, accel_sample_interval, check_landing_interval, check_override_interval])
flight_log.write("\nThe rocket has landed")
flight_log.write("\nTime (ms): " + str(time_queue.peek()))
uart.write("Landing Detected")
#-----LANDED PHASE-----

#DO STUFF AFTER LANDING HERE
#Do stuff to make sure these close no matter what

servo = machine.PWM(machine.Pin(18))
servo.freq(50) #Set PWM frequency

#define stop,CCW and CW timing in ns

#Our servo has a max ns of 2000000, min ns 1000000, and stops at ns 1500000
servoStop= 1500000
servoCCW = 1450000
servoCW = 1550000

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
init_adjust_time = time.ticks_ms()
def adjust_servo_angle():
    if time.ticks_diff(time.ticks_ms(), init_adjust_time) > 10000:
        raise StopIteration
    if(is_not_vertical()): #check that rocket is not moving and level
        if imu.euler()[2] > 3: #Leveling code
            servo.duty_ns(servoCCW)
        elif imu.euler()[2] < -3:
            servo.duty_ns(servoCW)
        else:
            servo.duty_ns(servoStop) #Tell servo to stop if the BNO is level 
print("program running")
servo.duty_ns(servoStop) #Servo is turned off initially

do_every([write_to_imu_data, adjust_servo_angle], [imu_data_interval, 10])
print("Done")
servo.duty_ns(servoStop) #Servo is turned off initially
imu_data.close()
flight_log.close()


