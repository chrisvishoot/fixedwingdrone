#**************************************************************
#               Imports
#**************************************************************

import time
import threading
import traceback
import RPi.GPIO as GPIO
from datetime import datetime
from GPS_serial import GPS_AdafruitSensor
from L3GD20 import L3GD20
from Adafruit_LSM303 import Adafruit_LSM303
from Adafruit_BMP085 import BMP085
#**************************************************************
#               Global Variables
#**************************************************************

exception_file = open('log_files/expection_report.txt', 'w')
exception_file.close()

#Set Altimeter
i2c_data_avail = False
temp = 0
pressure = 0
altimeter_and_temp = BMP085(debug = False)
initial_altitude = altimeter_and_temp.readAltitude()
altitude = initial_altitude

#Set up Accelerometer & magnitomiter
accel = Adafruit_LSM303()
ax = 0
ay = 0
az = 0
magx = 0
magy = 0
magz = 0
orientation = 0

# GPS setup
gps_data_avail = False
gps_sensor = GPS_AdafruitSensor(debug=True, interface='/dev/ttyAMA0')
(fix, lat, lon, heading, speed, gps_altitude, num_sat, timestamp, datestamp) = gps_sensor.read_sensor()
fix = 0
lat = 0
lon = 0
heading = 0
speed = 0
gps_altitude = 0
num_sat = 0
timestamp = 0
datestamp = 0

# Accellerometer set up
Gyro = L3GD20(busId = 1, slaveAddr = 0x6b, ifLog = False, ifWriteBlock=False)
Gyro.Set_PowerMode("Normal")
Gyro.Set_FullScale_Value("250dps")
Gyro.Set_AxisX_Enabled(True)
Gyro.Set_AxisY_Enabled(True)
Gyro.Set_AxisZ_Enabled(True)
Gyro.Init()
Gyro.Calibrate()
wx = 0
wy = 0
wz = 0

#**************************************************************
#               GPIO Start Button Initialization
#
#       This is used for starting and stopping the recorder
#       Recoder will only start then stop. Must restart the
#       program to start over.
#**************************************************************
exit_program = False
start = False
last_interrupt = datetime.now()
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(21, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def go(channel):
	global last_interrupt
	global start
	global exit_program
	t0 = datetime.now()
        tf2 = datetime.now()
        delta_t = tf2 - t0
        
	tf1 = datetime.now() - last_interrupt
	if tf1.seconds >= 1:
		last_interrupt = datetime.now()
		start = not start
		if start:
                        print('Start')
                else:
                        print('Stop')
        while not GPIO.input(21):
                tf2 = datetime.now()
        delta_t = tf2 - t0
	if delta_t.seconds > 2:
                exit_program = True

GPIO.add_event_detect(21,GPIO.FALLING)
GPIO.add_event_callback(21, go)

# Recording indicator
GPIO.setup(5,GPIO.OUT)
GPIO.output(5,0)

#**************************************************************
#               GPIO interrupt for recording pwm
#**************************************************************
channel_1o = datetime.now()
channel_2o = datetime.now()
channel_3o = datetime.now()
channel_4o = datetime.now()
channel_1 = datetime.now() - channel_1o  
channel_2 = datetime.now() - channel_2o  
channel_3 = datetime.now() - channel_3o  
channel_4 = datetime.now() - channel_4o  


GPIO.setup(4, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


#**************************************************************
#       Wait for the start button to be pressed.
#**************************************************************
while not start and not exit_program:
	print('Waiting')
	GPIO.output(5,1)
        time.sleep(.05)
	GPIO.output(5,0)
	time.sleep(.05)
print('Going\n')
GPIO.output(5,1)	
#**************************************************************
#       Threads for getting data from peripherals
#**************************************************************

exit_thread = False

def read_I2C_sensors():
	global i2c_data_avail, exit_thread 
	global s
	global ax, ay, az
	global wx, wy, wz
	global magx, magy, magz, orientation
	global temp, pressure, altitude
	while start and not exit_thread:
                try:
                        altitude = altimeter_and_temp.readAltitude()#initial_altitude)
                        pressure = altimeter_and_temp.readPressure()
                        temp = altimeter_and_temp.readTemperature()
                        (wx, wy, wz) = Gyro.Get_CalOut_Value()
                        # Orientation still needs to be calculated.
                        [[ax, ay, az],[magx, magy, magz, orientation]] = accel.read()
                        i2c_data_avail = True
                except:
                        exception_file = open('log_files/expection_report.txt', 'a')
                        exception_file.write(str(str(datetime.now()) + '\n'))
                        traceback.print_exc(file=exception_file)
                        exception_file.write('\n\n')
                        exception_file.close()

def GPS_logging():
	global gps_data_avail
        global fix, lat, lon, heading, speed, gps_altitude, num_sat, timestamp, datestamp
	while start and not exit_thread:
                try:
                        (fix, lat, lon, heading, speed, gps_altitude, num_sat, timestamp, datestamp) = gps_sensor.read_sensor()

		except:
                        print('Exception')
                        exception_file = open('log_files/expection_report.txt', 'a')
                        exception_file.write(str(str(datetime.now()) + '\n'))
                        traceback.print_exc(file=exception_file)
                        exception_file.write('\n\n')
                        exception_file.close()
                        time.sleep(1)

                time.sleep(1)
                gps_data_avail = True


def Measure_PWM():
        global channel_1o
        global channel_2o
        global channel_3o
        global channel_4o
        global channel_1 
        global channel_2 
        global channel_3  
        global channel_4
        count = 0
        while start and not exit_thread:
                print(count)
                count = count + 1
                GPIO.wait_for_edge(4,GPIO.RISING)
                channel_1o = datetime.now()
                GPIO.wait_for_edge(4,GPIO.FALLING)
                channel_1 = datetime.now() - channel_1o
                
                GPIO.wait_for_edge(17,GPIO.RISING)
                channel_2o = datetime.now()
                GPIO.wait_for_edge(17,GPIO.FALLING)
                channel_2 = datetime.now() - channel_2o 

                GPIO.wait_for_edge(27,GPIO.RISING)
                channel_3o = datetime.now()
                GPIO.wait_for_edge(27,GPIO.FALLING)
                channel_3 = datetime.now() - channel_3o 

                GPIO.wait_for_edge(22,GPIO.RISING)
                channel_4o = datetime.now()
                GPIO.wait_for_edge(22,GPIO.FALLING)
                channel_4 = datetime.now() - channel_4o 
#**************************************************************
#               Main Loop
#       The program executes from here.
#**************************************************************
recording_start_time = datetime.now()
initial_recording_time = datetime.now()
total_recording_time = datetime.now() - recording_start_time
while not exit_program:
        while start and not exit_program:
                file_start_time = datetime.now()
                file_running_time = datetime.now()- file_start_time
                logger = open(str('log_files/log_file_' + str(datetime.now()) + '.csv'), 'w')
                logger.write('recording_time, fix, lat, lon, heading, ')
                logger.write('speed, altitude, num_sat, ')
                logger.write('time_stamp, date_stamp, magx, magy, magz, ')
                logger.write('ax, ay, az, wx, wy, wz, temp, pressure, calculated altitude, ')
                logger.write('Channel 1 milliseconds, Channel 2 milliseconds, Channel 3 milliseconds, Channel 4 milliseconds\n')
                run_I2C_sensors = threading.Thread(target=read_I2C_sensors)
                run_GPS_logging = threading.Thread(target=GPS_logging)
                run_measure_PWM = threading.Thread(target=Measure_PWM)
                run_I2C_sensors.start()
                run_GPS_logging.start()
                run_measure_PWM.start()
                GPIO.output(5,1)
                ######################################
                #print('Outer Loop')
                ######################################
                while file_running_time.seconds <= 300 and start and not exit_program:
                        ######################################
                        #print(file_running_time.seconds)
                        ######################################
                        file_running_time = datetime.now() - file_start_time
                        while not i2c_data_avail and start:
                                time.sleep(0)
                        i2c_data_avail = False

                        #Write to output file with GPS data
                        if gps_data_avail:
                                total_recording_time = datetime.now() - initial_recording_time
                                gps_data_avail = False
                                #Write with GPS data
                                logger.write(str(str(total_recording_time) + ', ' + str(fix) + ', ' + str(lat) + ', ' + str(lon) + ', ' + str(heading) + ', '))
                                logger.write(str(str(speed) + ', ' + str(gps_altitude) + ', ' + str(num_sat) + ', '))
                                logger.write(str(str(timestamp) + ', ' + str(datestamp) + ', '))
                                #Accelerometer and Magentometer Data
                                logger.write(str(str(magx) + ', ' + str(magy) + ', ' + str(magz) + ', '))
                                logger.write(str(str(ax) + ', ' + str(ay) + ', ' + str(az) + ', '))
                                #Gyro Data
                                logger.write(str(str(wx) + ', ' + str(wy) + ', ' + str(wz) + ', '))
                                #Altimeter/Baro Pressure Data
                                logger.write(str(str(temp) + ', ' + str(pressure) + ', ' + str(altitude) + ', '))
                                #Recorder Channels
                                logger.write(str(str(channel_1.microseconds) + ', ' + str(channel_2.microseconds) + ', ' + str(channel_3.microseconds) + ', ' + str(channel_4.microseconds) + '\n'))
                                print(str(str(channel_1.microseconds) + ', ' + str(channel_2.microseconds) + ', ' + str(channel_3.microseconds) + ', ' + str(channel_4.microseconds) + '\n'))
                        #Write to output file without GPS data
                        else:
                                total_recording_time = datetime.now() - initial_recording_time
                                #GPS Data (Empty)
                                logger.write(str(str(total_recording_time) + ', , , , , , , , , , '))
                                #Accelerometer and Magentometer Data
                                logger.write(str(str(magx) + ', ' + str(magy) + ', ' + str(magz) + ', '))
                                logger.write(str(str(ax) + ', ' + str(ay) + ', ' + str(az) + ', '))
                                #Gyro Data
                                logger.write(str(str(wx) + ', ' + str(wy) + ', ' + str(wz) + ', '))
                                #Altimeter/Baro Pressure Data
                                logger.write(str(str(temp) + ', ' + str(pressure) + ', ' + str(altitude) + ', '))
                                #Recorder Channels
                                logger.write(str(str(channel_1.microseconds) + ', ' + str(channel_2.microseconds) + ', ' + str(channel_3.microseconds) + ', ' + str(channel_4.microseconds) + '\n'))
                
                logger.close()
                exit_thread = True
                run_I2C_sensors.join()
                run_GPS_logging.join()
                exit_thread = False
                count = 0
                ######################################
                print('Closing logger.')
                ######################################
                while count < 3:
                        count = count + 1
                        GPIO.output(5,1)
                        time.sleep(0.05)
                        GPIO.output(5,0)
                        time.sleep(0.05)
                        GPIO.output(5,1)
                        time.sleep(0.05)
                        GPIO.output(5,0)
                        time.sleep(0.05)
                        GPIO.output(5,1)
                        time.sleep(0.05)
                        GPIO.output(5,0)
                        time.sleep(1)
                        logger.close()
        while not start and not exit_program:
                print('Waiting in main loop')
                GPIO.output(5,1)
                time.sleep(.25)
                GPIO.output(5,0)
                time.sleep(.25)
        print('Going\n')
        


print('Exiting!')
count = 0
while count < 3:
        count = count + 1
        GPIO.output(5,1)
        time.sleep(0.1)
        GPIO.output(5,0)
        time.sleep(0.1)
        GPIO.output(5,1)
        time.sleep(0.1)
        GPIO.output(5,0)
        time.sleep(0.1)
        GPIO.output(5,1)
        time.sleep(0.1)
        GPIO.output(5,0)
        time.sleep(1)

	

