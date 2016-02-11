import time
import threading
import RPi.GPIO as GPIO
import time
from datetime import datetime
from GPS_serial import GPS_AdafruitSensor
from L3GD20 import L3GD20
from Adafruit_LSM303 import Adafruit_LSM303
from Adafruit_BMP085 import BMP085
#**************************************************************
#               Global Variables
#**************************************************************
i2c_data_avail = False
gps_data_avail = False
start = False
channel_1 = 0
channel_2 = 0
channel_3 = 0
channel_4 = 0
channel_1o = 0
channel_2o = 0
channel_3o = 0
channel_4o = 0
file_number = 0




last_interrupt = datetime.now()
t0 = datetime.now()
count = 0
dt = []


#Set Altimeter
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
magz = 0
magy = 0
magz = 0

# GPS setup
gps_sensor = GPS_AdafruitSensor(debug=True, interface='/dev/ttyAMA0')
(fix, lat, lon, heading, speed, altitude, num_sat, timestamp, datestamp) = gps_sensor.read_sensor()
fix = 0
lat = 0
lon = 0
heading = 0
speed = 0
altitude = 0
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
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)


GPIO.setup(21, GPIO.IN, pull_up_down=GPIO.PUD_UP)
def go(channel):
	global last_interrupt
	global start
	tf = datetime.now() - last_interrupt
	if tf.seconds >= 1:
		last_interrupt = datetime.now()
		start = not start
		if start:
                        print('Start')
                else:
                        print('Stop')
	
GPIO.add_event_detect(21,GPIO.FALLING)
GPIO.add_event_callback(21, go)

# Recording indicator
GPIO.setup(5,GPIO.OUT)
GPIO.output(5,0)

#**************************************************************
#               GPIO interrupt for recording pwm
#**************************************************************

GPIO.setup(4, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_UP)
def ch1(channel):
	global channel_1o
	global channel_1
	if GPIO.input(4):
		channel_1o = datetime.now().microsecond
	else:
		channel_1 = datetime.now().microsecond - channel_1o     
def ch2(channel):
	global channel_2o
	global channel_2
	if GPIO.input(17):
		channel_2o = datetime.now().microsecond
	else:
		channel_2 = datetime.now().microsecond - channel_2o
def ch3(channel):
	global channel_3o
	global channel_3
	if GPIO.input(27):
		channel_3o = datetime.now().microsecond
	else:
		channel_3 = datetime.now().microsecond - channel_3o
def ch4(channel):
	global channel_4o
	global channel_4
	if GPIO.input(22):
		channel_4o = datetime.now().microsecond
	else:
		channel_4 = datetime.now().microsecond - channel_4o
GPIO.add_event_detect(4,GPIO.BOTH)
GPIO.add_event_detect(17,GPIO.BOTH)
GPIO.add_event_detect(27,GPIO.BOTH)
GPIO.add_event_detect(22,GPIO.BOTH)
GPIO.add_event_callback(4, ch1)
GPIO.add_event_callback(17, ch2)
GPIO.add_event_callback(27, ch3)
GPIO.add_event_callback(22, ch4)

#**************************************************************
#       Wait for the start button to be pressed.
#**************************************************************
while not start:
	print('Waiting')
	GPIO.output(5,1)
        time.sleep(.25)
	GPIO.output(5,0)
	time.sleep(.25)
print('Going\n')
GPIO.output(5,1)	
#**************************************************************
#       Threads for getting data from peripherals
#**************************************************************

def read_I2C_sensors():
	global i2c_data_avail
	global s
	global ax, ay, az
	global wx, wx, wz
	global magx, magy, magz
	global temp, pressure, altitude
	while start:
                altitude = altimeter_and_temp.readAltitude(initial_altitude)
                pressure = altimeter_and_temp.readPressure()
                temp = altimeter_and_temp.readTemperature()
                (wx, wy, wz) = Gyro.Get_CalOut_Value()
		i2c_data_avail = True
                

def GPS_logging():
	global gps_data_avail
        global fix, lat, lon, heading, speed, altitude, num_sat, timestamp, datestamp
	while start:
		(fix, lat, lon, heading, speed, altitude, num_sat, timestamp, datestamp) = gps_sensor.read_sensor()
		gps_data_avail = True
		time.sleep(2)






#**************************************************************
#               Main Loop
#       The program executes from here.
#**************************************************************

while True:
        logger = open(str('log_file_' + str(file_number) + '.txt'), 'w')
        logger.write('fix, lat, lon, heading, ')
        logger.write('speed, altitude, num_sat, ')
        logger.write('time_stamp, date_stamp\n')
        run_I2C_sensors = threading.Thread(target=read_I2C_sensors)
        run_GPS_logging = threading.Thread(target=GPS_logging)
        run_I2C_sensors.start()
        run_GPS_logging.start()
        while start:
                while not i2c_data_avail and start:
                        time.sleep(0)
                i2c_data_avail = False
                if gps_data_avail:
                        #print('Printing with GPS Data.\n')
                        gps_data_avail = False
                        #Write with GPS data
                        logger.write(str(str(fix) + ', ' + str(lat) + ', ' + str(lon) + ', ' + str(heading) + ', '))
                        logger.write(str(str(speed) + ', ' + str(altitude) + ', ' + str(num_sat) + ', '))
                        logger.write(str(str(timestamp) + ', ' + str(datestamp) + '\n'))
                else:
                        #print('Printing without GPS Data.\n')
                        #logger.write(str(count) + ' Printing without GPS Data.\n')
                        time.sleep(0)
        GPIO.output(5,0)
        logger.close()
        run_I2C_sensors.join()
        run_GPS_logging.join()
        file_number = file_number + 1
        running_time = datetime.now() - t0
        print('Complete: ' + str(running_time.seconds) + ' seconds')
        while not start:
                print('Waiting in main loop')
                GPIO.output(5,1)
                time.sleep(.25)
                GPIO.output(5,0)
                time.sleep(.25)
        print('Going\n')
        GPIO.output(5,1)

	

