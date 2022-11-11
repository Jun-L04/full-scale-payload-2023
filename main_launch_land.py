import machine
import time
from bno055 import *
from queue import *#do Queue(max_size, threshold) to make object
from math import floor
from sys import exit

from bno055_base import BNO055_BASE
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
    print("Size of queue: " + str(size))
    arr = queue.get_array()
    print("Length of list: " + str(len(arr)))
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
    assert(len(func_arr) == len(interval_arr))
    prevTimes = [-999999]*len(func_arr)
    while True:
        for i, func in enumerate(func_arr):
            if(time.ticks_ms() - prevTimes[i] > interval_arr[i]):
                try:
                    func()
                    prevTimes[i] = time.ticks_ms()
                except StopIteration:#Leave the while loop
                    return

# Pico: hard I2C doesn't work without this patch
# https://github.com/micropython/micropython/issues/8167#issuecomment-1013696765
i2c = machine.I2C(0, sda=machine.Pin(16), scl=machine.Pin(17))  # EIO error almost immediately

# All platforms: soft I2C requires timeout >= 1000μs
# i2c = machine.SoftI2C(sda=machine.Pin(16), scl=machine.Pin(17), timeout=1_000)
# ESP8266 soft I2C
# i2c = machine.SoftI2C(scl=machine.Pin(2), sda=machine.Pin(0), timeout=100_000)
# ESP32 hard I2C
# i2c = machine.I2C(1, scl=machine.Pin(21), sda=machine.Pin(23))
imu = BNO055(i2c)

#Declaring queues. They must be accessible via the global context
time_queue = None
accel_queue = None
#Inputs: Interval: How much acceleration data should be kept track of (eg last 2000 ms, interval = 2000)
#						NOTE: This does not mean acceleration is updated every interval ms
#Output: A function that can be passed into do_every to update the time and acceleration queues
def make_data_updater(interval):
    def updater():
        cur_time = time.ticks_ms()
        time_queue.enqueue(cur_time)
        accel_data = imu.accel()
        accel_queue.enqueue((accel_data[0]**2+accel_data[1]**2+accel_data[2]**2)**.5) 
        while cur_time - time_queue.peek() > interval:
            time_queue.dequeue()
            accel_queue.dequeue()
    return updater

#-----CALIBRATION PHASE-----
def calibration_fn():
    print('Calibration required: sys {} gyro {} accel {} mag {}'.format(*imu.cal_status()))
    if imu.calibrated():
        print("CALIBRATED!");
        print(imu.sensor_offsets())
        #bytearray(b'\xfa\xff\x00\x00\xe9\xffF\x04\x13\x01|\xff\xff\xff\x00\x00\x00\x00\xe8\x03\xec\x01')
        raise StopIteration
#Check for calibration every 1000 ms
######------do_every([calibration_fn], [1000])
#The IMU is calibrated at this point
#The reference gravity must be calculated in setup for the rest of the program to use
#For stopping the executition of program during testing
offset_arr = bytearray(b'\xfa\xff\x00\x00\xe9\xffF\x04\x13\x01|\xff\xff\xff\x00\x00\x00\x00\xe8\x03\xec\x01')
imu.set_offsets(offset_arr)

print("IMU is calibrated")
print("Time (ms): " + str(time.ticks_ms()))


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
    if time.ticks_ms() - time_queue.peek() < check_interval*0.5:
        return
    stats = mean_variance(accel_queue)
    if stats[1] < 0.1 and stats[0] > 9.5 and stats[0] < 10.1:#ARBITRARY: if variance less than .1 m/s^2, also check that mean is reasonable (g+-.3)
        #May also want to check that queue has big enough number of elements
        #Set reference gravity
        print("FINAL: " + str(stats))
        GRAVITY = stats[0]*1.05 #ARBITRARY: Multiply by 1.1 to give some room for error
        raise StopIteration
do_every([data_updater, find_reference_gravity], [accel_sample_interval, check_interval])
#GRAVITY is set at this point
print("Gravity has been set: " + str(GRAVITY))
print("Time (ms): " + str(time_queue.peek()))

#-----BEFORE FLIGHT PHASE-----
#Reset time and acceleration queues
queue_frequency = 50 #Hz
burn_time = 2970 * .75#Multiply by .75 for some room for error
accel_sample_interval = 1000 / queue_frequency #calculate time between samples in ms based on desired frequency
max_size = calculate_max_size(queue_frequency, burn_time)

time_queue = Queue(max_size, None)
accel_queue = Queue(max_size, GRAVITY)

def check_launch():
    if time.ticks_ms() - time_queue.peek() < 0.5*burn_time:
        return#Dont check for launch if there's not enough data in the queue yet
    if accel_queue.get_proportion_above_threshold() > 0.95:#ARBITRARY: If 95% of the data is above the gravity threshold, then check variance
        if mean_variance(accel_queue)[1] > .3:#ARBITRARY: If the variance is above 0.5 m/s^2, then launch is detected
            raise StopIteration

data_updater = make_data_updater(burn_time)#Function to update accelerations, keeping only last burn_time ms in the queue


check_launch_interval = 1000 #ARBITRARY: check for launch every check_launch_interval ms
do_every([data_updater, check_launch], [accel_sample_interval, check_launch_interval])
#The rocket has launched at this point
print("The rocket has launched")
print("Time (ms): " + str(time_queue.peek()))

#-----IN FLIGHT PHASE-----
#Reset time and acceleration queues
queue_frequency = 25 #Hz
check_interval = 10000 #Want to keep track fo last 10 seconds of acceleration data
accel_sample_interval = 1000 / queue_frequency #calculate time between samples in ms based on desired frequency
max_size = calculate_max_size(queue_frequency, check_interval)

time_queue = Queue(max_size, None)
accel_queue = Queue(max_size, GRAVITY)

def check_landing():
    if time.ticks_ms() - time_queue.peek() < 0.5*check_interval:
        return#Dont check for landing if there's not enough data in the queue yet
    if accel_queue.get_proportion_above_threshold() < 0.05:#ARBITRARY: If no more than 5% of the acceleration data is above the gravity threshold, then check variance
        if mean_variance(accel_queue)[1] < .25:#ARBITRARY: If variance is below 0.25 m/s^2, then landing is detected
            raise StopIteration
data_updater = make_data_updater(check_interval)
check_landing_interval = 3000#ARBITRARY: Check for landing every 3 seconds
do_every([data_updater, check_landing], [accel_sample_interval, check_landing_interval])
print("The rocket has landed")
print("Time (ms): " + str(time_queue.peek()))
#-----LANDED PHASE-----

#DO STUFF AFTER LANDING HERE


#print('Time: {:5.2f}\nAccel Magnitude: {:5.2f}'.format(sensor_time, accel_queue))
#print('Temperature {}°C'.format(imu.temperature()))
#print('Mag       x {:5.0f}    y {:5.0f}     z {:5.0f}'.format(*imu.mag()))
#print('Gyro      x {:5.0f}    y {:5.0f}     z {:5.0f}'.format(*imu.gyro()))
#print('Accel     x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*accel_data))
#print('Lin acc.  x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.lin_acc()))
#print('Gravity   x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.gravity()))
#print('Heading     {:4.0f} roll {:4.0f} pitch {:4.0f}'.format(*imu.euler()))
"""
#------TESTING DATA--------
GRAVITY = 9.94
queue_frequency = 10 #Hz
burn_time = 1550#Multiply by .95 for some room for error
accel_sample_interval = 1000 / queue_frequency #calculate time between samples in ms based on desired frequency
max_size = calculate_max_size(queue_frequency, burn_time)+1000

time_queue = Queue(max_size, None)
accel_queue = Queue(max_size, GRAVITY)

#---insert time and accel data here---
testing_ind = 0
def data_updater():
    global testing_ind
    interval = 1550
    cur_time = testing_time[testing_ind]
    time_queue.enqueue(cur_time)
    print(cur_time)
    accel_queue.enqueue(testing_accel[testing_ind])
    testing_ind += 1
    while cur_time - time_queue.peek() > interval:
        time_queue.dequeue()
        accel_queue.dequeue()

def check_launch():
    if testing_time[testing_ind] - time_queue.peek() < 0.5*burn_time:
        return#Dont check for launch if there's not enough data in the queue yet
    print(accel_queue.get_proportion_above_threshold())
    if accel_queue.get_proportion_above_threshold() > 0.95:#ARBITRARY: If 95% of the data is above the gravity threshold, then check variance
        if mean_variance(accel_queue)[1] > .5:#ARBITRARY: If the variance is above 0.5 m/s^2, then launch is detected
            print(time_queue.peek())
            raise StopIteration
#-------END TESTING DATA------
"""