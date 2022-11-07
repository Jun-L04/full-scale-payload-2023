#XBEE Serial Communications Demo 11-6-2022
#Written by Anton Voronov (ravenspired) for the UMass Rocket Team
#Demonstration code for subscale rocket flight (Testing the successful transmission of data during flight of the rocket.
#Sends a transmission including made up accelerometer data and a timestamp of seconds since launch.
#Currently tested on an Adafruit Feather Huzzah (4MB Flash) running Micropython 1.19.1, connected to a sparkfun XBee Pro 538 Breakout Board.
#This script sends a transmission every second with the amount of seconds since program was run and with made up accelerometer data.

#WIRING DIAGRAM:
#Feather Board => XBee Pro 538
#3V => 3.3V
#GND => GND
#2 => DIN


import uos, machine
import gc
import time
import random
random.seed(1)
start_time = time.time()
gc.collect()

baud_rates = [110,300,600,1200,2400,4800,9600,14400,19200,38400,57600,115200,128000,256000]
baud_rate_selection = 6 #To change baud rate

from machine import UART
 
uart = UART(1, baud_rates[baud_rate_selection]) # setup uart object
uart.init(9600, parity=None, stop=1) # initialize the serial connection with given parameters
time.sleep(0.5)
uart.write('Initial Transmission - Rocket was connected to power') # How to send the message
time.sleep(0.5)

def transmit_data(content):
    uart.write('\n')
    uart.write('===Begin Transmission===\n')
    uart.write('seconds since power was connected:' + get_time_stamp() + '\n')
    uart.write(str(content)+'\n')
    uart.write('===End Transmission===\n')
    time.sleep(0.1)

def invent_accel_data(): # makes up accelerometer data and returns it in the format: "x=int, y=int, z=int"
    return str('x='+ str(random.getrandbits(8)) + ', y=' + str(random.getrandbits(8)) + ', z=' + str(random.getrandbits(8)))

def get_time_stamp(): # returns the amount of seconds since program started to run
    return str(time.time() - start_time)

while True:
    transmit_data(invent_accel_data()) 
    time.sleep(0.9)

