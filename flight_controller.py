#---------------------------------------------------------------
# 						Imports
#---------------------------------------------------------------

import time
from datetime import datetime
from L3GD20 import L3GD20
from Adafruit_LSM303 import Adafruit_LSM303
import math
from Adafruit_PWM_Servo_Driver import PWM
import threading
import RPi.GPIO as GPIO

#---------------------------------------------------------------
# 						imu setup
#---------------------------------------------------------------

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

accel = Adafruit_LSM303()
ax = 0
ay = 0
az = 0
magx = 0
magy = 0
magz = 0
orientation = 0
conv = 180/3.141592
roll_angle = 0
pitch_angle = 0
yaw_angle = 0

#---------------------------------------------------------------
# 						Servo Drive setup
#---------------------------------------------------------------

#pwm channels
ail=0
ele=1
rud=2
ttl=3
pan=4
tlt=5
ttl_min = 260
#pwm setup
pwm = PWM()
pwm.setPWMFreq(43)
pwm.setPWM(ail,1,275)#Aileron, RollL=200, RollR=400, Center=275
pwm.setPWM(ele,1,270)#Elevator, 400 pitch down, 200 pitch up
pwm.setPWM(rud,1,290)#Rudder, YawL=400, YawR=200, Center=290
pwm.setPWM(ttl,1,ttl_min)#Throttle
pwm.setPWM(pan,1,290)#Pan, PanL=400, PanR=200, Center=290
pwm.setPWM(tlt,1,250)#Tilt, TiltD=400, TileU=200, Center=250

#---------------------------------------------------------------
# 						GPIO Aknowledge/Request
#---------------------------------------------------------------

# Pin definitions
cc_akn_pin = 27
cc_req_pin = 22
auto_akn_pin = 4
auto_req_pin = 17

# Mode control variables
auto_mode = False
cc_mode = False

# Pin set up
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(cc_req_pin, GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(auto_req_pin, GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(cc_akn_pin, GPIO.OUT)
GPIO.setup(auto_akn_pin, GPIO.OUT)

def cc_req(channel):
	print("Changing CC Mode")
	if(GPIO.input(cc_req_pin)):
		print("CC Mode on")
		GPIO.output(cc_akn_pin,1)
		cc_mode = True
	else:
		GPIO.output(cc_akn_pin,0)
		print("CC Mode off")
		cc_mode = False

def auto_req(channel):
	
	if(GPIO.input(auto_req_pin)):
		print("Auto Mode on")
		GPIO.output(auto_akn_pin,1)
		auto_mode = True
	else:
		print("Auto Mode off")
		GPIO.output(auto_akn_pin,0)
		auto_mode = False



GPIO.add_event_detect(cc_req_pin,GPIO.BOTH)
GPIO.add_event_callback(cc_req_pin, cc_req)
GPIO.add_event_detect(auto_req_pin,GPIO.BOTH)
GPIO.add_event_callback(auto_req_pin, auto_req)
#---------------------------------------------------------------
# 						IMU & Control Thread
#---------------------------------------------------------------
run_threads = True


def control():
	global roll_angle, pitch_angle, yaw_angle,ail, ele, rud, ttl, pan, tlt, cc_mode, auto_mode, pwm, ttl_min, run_threads
	roll_error = 275
	pitch_error = 275
	ttl_output = ttl_min
	ttl_last_out = ttl_min
	last_pitch_error = roll_error 
	last_roll_error = pitch_error 
	print("Control Running")
	while run_threads:
		time.sleep(.1)
		
		ttl_last_out = ttl_output
		#ttl_output = ttl_min+4*pitch_angle
		ttl_output = 280
		if(ttl_output > 400):
			print("pitch_angle high: " + str(pitch_angle))
			ttl_output = 400
		elif(ttl_output < ttl_min):
			print("pitch_angle low: " + str(pitch_angle))
			ttl_output = ttl_min
		if(cc_mode):
			time.sleep(0)			
		elif(auto_mode):
			last_pitch_error = pitch_error
			pitch_error = 4(270 - pitch_angle)
			last_roll_error = roll_error
			roll_error = 4(270 - roll_angle)
			
			if (pitch_error > 400):
				pitch_error = 400
			elif(pitch_error < 200):
				pitch_error = 200
			if(roll_error > 400):
				roll_error = 400
			elif(roll_error < 200):
				roll_error = 200
		
		if(abs(last_roll_error - roll_error) > 10):
			print("Setting roll to: " + str(roll_error))
			pwm.setPWM(ail,1, int(roll_error))#Aileron, RollL=200, RollR=400, Center=275
		if(abs(last_pitch_error - pitch_error) > 10):
			print("Setting pitch to: " + str(pitch_error))
			pwm.setPWM(ele,1,int(pitch_error))#Elevator, 400 pitch down, 200 pitch up
		#pwm.setPWM(rud,1,290)#Rudder, YawL=400, YawR=200, Center=290
		if(abs(ttl_last_out - ttl_output) > 10):
			print("Setting ttl_output to: " + str(ttl_output))
			pwm.setPWM(ttl,1,int(ttl_output))#Throttle
		#pwm.setPWM(pan,1,290)#Pan, PanL=400, PanR=200, Center=290
		#pwm.setPWM(tlt,1,250)#Tilt, TiltD=400, TileU=200, Center=250

def imu():
	print("IMU Running")
	global roll_angle, pitch_angle, yaw_angle, run_threads
	while run_threads:
		time.sleep(.9)
		(wx, wy, wz) = Gyro.Get_CalOut_Value()
		[[ax, ay, az],[magx, magy, magz, orientation]] = accel.read()
		heading = (math.atan2(magy,magx)* 180)/3.141592		
		roll_angle = math.atan2(ay,az)
		pitch_in = -ax/((ay*math.sin(roll_angle)) + (az*math.cos(roll_angle)))
		pitch_angle = math.atan(pitch_in)
		yaw_y = (magz*math.sin(roll_angle)) - (magy*math.cos(roll_angle))   
		yaw_x = (magx*math.cos(pitch_angle)) + (magy*math.sin(pitch_angle)*math.sin(roll_angle)) + (magz*math.sin(pitch_angle)*math.cos(roll_angle))
		yaw_angle = math.atan2(yaw_y,yaw_x)
		pitch_angle = -round(pitch_angle * conv,2)
		roll_angle = round(roll_angle * conv,2)
		yaw_angle = round(yaw_angle * conv,2)
run_imu = threading.Thread(target=imu)
run_imu.start()
run_control = threading.Thread(target=control)
run_control.start()

#---------------------------------------------------------------
# 						Main loop
#---------------------------------------------------------------

while True:
	try:
		time.sleep(0)
	except KeyboardInterrupt:
		run_threads = False
		run_imu.exit()
		run_control.exit()
