import os
import machine
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
vfs = os.VfsFat(sd)
os.mount(vfs, "/sd")
#Input: path: file name including path such as "/sd/imu_data" (don't include extension or dot)
#		ext: extension such as "csv" (don't include dot)
def get_valid_file_name(path, ext):
    while True:
        try:
            os.stat(path)
            return path + "." + ext
        except OSError:
            path += 1

imu_data = open(get_valid_file_name("/sd/imu_data", "csv") , "w")#Need to close