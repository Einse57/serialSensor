#!/usr/bin/python
#
# An example two way message logging bridge between an Arduino-based sensor system and a host computer over USB serial
# The application demonstrates two way connection handshaking, rate parameter passing, and condition handling 
#

import serial
import struct
import time
import datetime
from datetime import datetime

# Rate parameters (in usecs) Max rate = 1470Hz based on current loop time:
adc_rate = 15625    # 64Hz = 1/64 = 0.015625 seconds = 15625 microseconds
imu_rate = 20000    # 50 Hz, 1/50 = 0.02 seconds = 20000 microseconds  

heartbeat = 0
sensor_serial = 0

# Windows paths
serial_path = 'COM10'
log_path = 'logfiles/'

# Linux paths
#serial_path = '/dev/ttyACM0'
#log_path = './logfiles/'

# serial_init()
# Open com port to sensor controller
# On most linux installs, Arduino shows up as a ttyACMx device
# You'll need to add the username to dialout group to make sure you can access this port
# sudo usermod -a -G dialout yourUserName
def serial_init():
    global sensor_serial
    try:
        if sensor_serial == 0:
            #print("[ ] Serial init first open")
            sensor_serial = serial.Serial(serial_path, 115200, timeout=1)
            sensor_serial.flushInput()
            return True
        else:
            if sensor_serial.isOpen():
                #print("[ ] Serial port already open")
                sensor_serial.close()
                sensor_serial = serial.Serial(serial_path, 115200, timeout=1)
                sensor_serial.flushInput()
                #print("[ ] Closed and reopened serial port")
                return True
            else:   
                #print("[ ] Serial port closed...Reopening serial port")
                sensor_serial = serial.Serial(serial_path, 115200, timeout=1)
                sensor_serial.flushInput()
                return True
    except:
        print("[!] Serial exception in serial_init")
        return False

# sensor_init()
# Initialize sensor controller
def sensor_init():

    global adc_rate
    global imu_rate
    global sensor_serial      
    
    # Look for handshake bytes from Sensor
    print("[ ] Waiting for handshake from sensor controller")
    
    while True:
        data = sensor_serial.read(1) 
        if data == b'\xAA':
            print("[X] Got response from sensor controller")
            #print (data)
            break

    # Send handshake byte to Arduino
    sensor_serial.write(b'\xBB')

    # Send rate information to Arduino
    # Arduino expects rate as a four byte unsigned long
    
    # Send adc rate
    byte0 = adc_rate & 0x000000FF #LSB
    byte1 = (adc_rate >> 8) & 0x000000FF
    byte2 = (adc_rate >> 16) & 0x000000FF
    byte3 = (adc_rate >> 24) & 0x000000FF #MSB
    sensor_serial.write(struct.pack('4B', byte3, byte2, byte1, byte0))
    #Send imu rate
    byte0 = imu_rate & 0x000000FF #LSB
    byte1 = (imu_rate >> 8) & 0x000000FF
    byte2 = (imu_rate >> 16) & 0x000000FF
    byte3 = (imu_rate >> 24) & 0x000000FF #MSB
    sensor_serial.write(struct.pack('4B', byte3, byte2, byte1, byte0))
    # Read back rate info
    time.sleep(.5) # give some time to turn around
    readback = struct.unpack('<LL', sensor_serial.read(8))
    print ("[ ] Rate loop intervals (IMU, ADC): " + str(readback))
    return

# End of sensor_init()
    
# sensor_read()
# This performs a blocking read of sensor controller serial data
# Could also make is non-blocking
def sensor_read():

    global heartbeat
    
    msgID = sensor_serial.read(1)
    # Parse ResMed message format
    if msgID == b'\xCC':
        #print("Got an ADC message")
        #timestamp(4B), ADC0(2B), ADC1(2B), ADC2(2B), ADC3(2B)
        sensor_values = struct.unpack('<LHHHH', sensor_serial.read(12))
        log_cache.append("ADC, " + "%d, %d, %d, %d, %d\n" % (sensor_values[0], sensor_values[1], sensor_values[2], sensor_values[3], sensor_values[4]))
        #print(sensor_values)
        heartbeat = 0
        return sensor_values
    # Parse IMU message format
    if msgID == b'\xDD':
        #print("Got an IMU message")
        #timestamp(4B), AccX(2B), AccY(2B), AccZ(2B), MagX(2B), MagY(2B), MagZ(2B), GyroX(2B), GyroY(2B), GyroZ(2B)
        sensor_values = struct.unpack('<Lhhhhhhhhh', sensor_serial.read(22))
        log_cache.append("IMU, " + "%d, %f, %f, %f, %f, %f, %f, %f, %f, %f\n" % (sensor_values[0], sensor_values[1]*0.00390625, sensor_values[2]*0.00390625, sensor_values[3]*0.00390625, \
                                                                                                   sensor_values[4]*0.92, sensor_values[5]*0.92, sensor_values[6]*0.92, \
                                                                                                   sensor_values[7]/14.375, sensor_values[8]/14.375, sensor_values[9]/14.375))                                    
            
        #print(sensor_values)
        heartbeat = 0
        return sensor_values
    # No/Unexpected Data Received
    # Record the event in the log then attempt to reestablish comms
    else:
        heartbeat = heartbeat + 1
        if heartbeat > 20:
            log_file.write("[!] Sensor Comms Error " + str(today.strftime("%b-%d-%y_%H%M%S")) + "\n")
            while(not(serial_init())):
                time.sleep(1)
            print("[!] Error Communicating with Sensor...Trying Handshake Again")
            sensor_init()
            heartbeat = 0
            log_file.write("[X] Sensor Comms Reestablished " + str(today.strftime("%b-%d-%y_%H%M%S")) + "\n")
            print("[X] Sensor Comms Reestablished")
                
# End of sensor_read()

### Start main program work ###

# Open log file
today = datetime.now()
log_file = open(log_path + str(today.strftime("%b-%d-%y_%H%M%S")) + ".txt", "w")

log_file.write("Sensor Log\n")
log_file.write("ADC_RATE, " + str(resmed_rate) + ", IMU_RATE, " + str(imu_rate) + "\n")
log_file.write("IMU Format, [Timestamp(usecs)], [ADXL345 AccelX(g)], [ADXL345 AccelY(g)], [ADXL345 AccelZ(g)], [HMC5883L MagX(mGauss)], [HMC5883L MagY(mGauss)], [HMC5883L MagZ(mGauss)], [ITG3200 GyroX(deg/sec)], [ITG3200 GyroY(deg/sec)], [ITG3200 GyroZ(deg/sec)]\n")
log_file.write("ADC Format, [Timestamp(usecs)], [ADC0], [ADC1], [ADC2], [ADC3]\n")

log_cache = []

# Init devices
# Initialize serial port
print ("[ ] Starting serial port init...")
while(not(serial_init())):
    time.sleep(1)
print ("[X] Serial port init complete")

# Initial handshake
sensor_init()

print ("[X] Startup success...Logging")
print ("[ ] CTL+C to exit...")

while True:
    try:
        # Read data from sensor
        sensor_readings = sensor_read()
        # Maintain the log
        if (len(log_cache) >= 1600): # new setting = 1600 (25 seconds at 64Hz), old setting = 160000 (41 minutes at 64Hz)
            log_file.write(''.join(log_cache))
            log_cache = []
        
    except KeyboardInterrupt:
        log_file.write(''.join(log_cache))
        log_file.close()
        sensor_serial.close()
        print("[X] Closed log and sensor TTY port")
        break












