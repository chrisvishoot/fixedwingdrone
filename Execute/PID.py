from pinAssignments import *
import numpy as np
import time
import sys
from L3GD20 import L3GD20
from Adafruit_LSM303 import Adafruit_LSM303
from Adafruit_PWM_Servo_Driver import PWM
import math
import os
from gps import *
from time import *
import threading
from heading import *
import serial
import RPi.GPIO as GPIO


#---------------------------------------------------------------
# 						GPIO Aknowledge/Request
#---------------------------------------------------------------


#If both the auto and cc are false / off, then we will assume manual for our logic





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
GPIO.setup(uart_toggle, GPIO.OUT) # Toggle between gps and xbee

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




class PID:
	"""
	Discrete PID control
	"""

	def __init__(self, P=1, I=0.0, D=1, Derivator=0, Integrator=0, Integrator_max=500, Integrator_min=-500):

		self.Kp=P
		self.Ki=I
		self.Kd=D
		self.Derivator=Derivator
		self.Integrator=Integrator
		self.Integrator_max=Integrator_max
		self.Integrator_min=Integrator_min

		self.set_point=0.0
		self.error=0.0

	def update(self,current_value):
		"""
		Calculate PID output value for given reference input and feedback
		"""

		self.error = self.set_point - current_value

		self.P_value = self.Kp * self.error
		self.D_value = self.Kd * ( self.error - self.Derivator)
		self.Derivator = self.error

		self.Integrator = self.Integrator + self.error

		if self.Integrator > self.Integrator_max:
			self.Integrator = self.Integrator_max
		elif self.Integrator < self.Integrator_min:
			self.Integrator = self.Integrator_min

		self.I_value = self.Integrator * self.Ki

		PID = self.P_value + self.I_value + self.D_value

		return PID

	def setPoint(self,set_point):
		"""
		Initilize the setpoint of PID
		"""
		self.set_point = set_point
		self.Integrator=0
		self.Derivator=0

	def setIntegrator(self, Integrator):
		self.Integrator = Integrator

	def setDerivator(self, Derivator):
		self.Derivator = Derivator

	def setKp(self,P):
		self.Kp=P

	def setKi(self,I):
		self.Ki=I

	def setKd(self,D):
		self.Kd=D

	def getPoint(self):
		return self.set_point

	def getError(self):
		return self.error

	def getIntegrator(self):
		return self.Integrator

	def getDerivator(self):
		return self.Derivator

def getPitchYawRoll(Gyro, Accel):
    conv = 180 / math.pi

    (wx, wy, wz) = Gyro.Get_CalOut_Value()
    #acceleration in the x, y and z
    [[ax, ay, az],[magx, magy, magz, orientation]] = Accel.read()
    #heading = (math.atan2(magy,magx)* 180)/ math.pi
    roll_angle = math.atan2(ay,az)
    pitch_in = -ax/((ay*math.sin(roll_angle)) + (az*math.cos(roll_angle)))
    pitch_angle = math.atan(pitch_in)
    yaw_y = (magz*math.sin(roll_angle)) - (magy*math.cos(roll_angle))
    yaw_x = (magx*math.cos(pitch_angle)) + (magy*math.sin(pitch_angle)*math.sin(roll_angle)) + (magz*math.sin(pitch_angle)*math.cos(roll_angle))
    yaw_angle = math.atan2(yaw_y,yaw_x)
    if(yaw_y > 0 and yaw_x < 0):
        yaw_angle = yaw_angle + math.pi
    elif(yaw_y < 0 and yaw_x < 0):
        yaw_angle = yaw_angle + math.pi

    heading = math.atan2(magz * math.sin(roll_angle) - magy*math.cos(roll_angle) ,
    magx * math.cos(pitch_angle) + magy * math.sin(pitch_angle) * math.sin(roll_angle) + magz * math.sin(pitch_angle) * math.sin(roll_angle))
    #(atan2(event.magnetic.y,event.magnetic.x) * 180) / Pi;
    #heading = (math.atan2(magy,magx) * 180) / math.pi;

    pitch_angle = round(pitch_angle * conv,2)
    roll_angle = round(roll_angle * conv,2)
    yaw_angle = round(yaw_angle * conv,2)
    #yaw = 180 * atan (accelerationZ/sqrt(accelerationX*accelerationX + accelerationZ*accelerationZ))/M_PI;
    yaw_angle = 180 * math.atan(az/math.sqrt(ax*ax + az*az))/math.pi;
    heading = (heading * 180) / math.pi
    #print yaw_angle
    return (pitch_angle, yaw_angle, roll_angle)



def mapValue(value, in_min=-45, in_max=45, out_min=195, out_max=410):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def restrictValues(value):
    if value < 195:
	return 195
    elif value > 410:
	return 410
    else:
	return value

gpsd = None #seting the global variable
prevLocation = None
heading = None

class GpsPoller(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
    global gpsd #bring it in scope
    gpsd = GPS(mode=WATCH_ENABLE) #starting the stream of info
    self.current_value = None
    self.running = True #setting the thread running to true

  def run(self):
    global gpsd
    while gpsp.running:
        try:
            prevLocation = (gpsd.fix.latitude, gpsd.fix.longitude)
            GPIO.output(uart_toggle, False)
            time.sleep(0.01) # Delay for 10 ms to make sure we're connected to the GPS
            gpsd.next() #this will continue to loop and grab EACH set of gpsd info to clear the buffer
            if (prevLocation is not None):
                heading = getTargetHeading(prevLocation, (gpsd.fix.latitude, gpsd.fix.longitude))
            time.sleep(1)
        except(StopIteration):
            pass

ser = serial.Serial("/dev/ttyAMA0")    #Open named port
ser.baudrate = 9600                     #Set baud rate to 9600, might have to change this

if __name__ == '__main__':
    Gyro = L3GD20(busId = 1, slaveAddr = 0x6b, ifLog = False, ifWriteBlock=False)
    Gyro.Set_PowerMode("Normal")
    Gyro.Set_FullScale_Value("250dps")
    Gyro.Set_AxisX_Enabled(True)
    Gyro.Set_AxisY_Enabled(True)
    Gyro.Set_AxisZ_Enabled(True)
    Gyro.Init()
    Gyro.Calibrate()

    Accel = Adafruit_LSM303()

    # gpsp = GpsPoller() # create the thread

    servoDriver = PWM(0x40)
    servoDriver.setPWMFreq(43)

    elevatorUp = 195
    elevatorDown = 410

    elevatorPid = PID()
    rollPid = PID()
    altitudePid = PID()
    rudderPid = PID()
    throttlePid = PID()

    pitch_angle, yaw_angle, roll_angle = getPitchYawRoll(Gyro, Accel)

    elevatorPid.setPoint(pitch_angle)
    rollPid.setPoint(roll_angle)
    rudderPid.setPoint(0) # Change 0 to "zero" for servo (value that's straight)
    throttlePid.setPoint(0) #Current throttle speed will be our normalization of the system.
    #-90 means that the plane is going up, or above the horizon
    # 90 means that the plane is going down, or below the horizon (relative to the current location of the plane)
    #if -90, then increase the speed.


    try:
        while True:
            pitch_angle, yaw_angle, roll_angle = getPitchYawRoll(Gyro, Accel)
            
            if(auto_mode):
                
                #FOR THE RUDDER PID ALGORITHM:
                #We need the Xbee for the target location, and currently this is not implemented yet
                #After Xbee is working, get the code from sensors / PID.py - sensors
                # Example target location sent through uart: "  47.244251, -122.438301, 124.5"
                #                                               (lat-11) ,  (long-11) , (alt-5)
                #                                                  (31 characters total)
                # Need to read in 31 characters for a target location

                throttlePidValue = throttlePid.update(pitch_angle)


                rollPidValue = rollPid.update(roll_angle)
                altitudePidValue = altitudePid.update(gpsd.fix.altitude)
                elevatorPid.setPoint(scaleAltitude(altitudePidValue))
                elevatorPidValue = elevatorPid.update(pitch_angle)

                print('Pitch: ' + str(pitch_angle) + '\tYaw: ' + str(yaw_angle) + '\tRoll: ' + str(roll_angle))
                #print yaw_angle
                print "PID Elevator: ", elevatorPidValue
                print "PID Roll: ", rollPidValue

                elevatorServoValue = int(restrictValues(mapValue(elevatorPidValue)))
                #def mapValue(value, in_min=-45, in_max=45, out_min=195, out_max=410):
                #throttle max = 400, min = 260

                throttleServoValue = int(restrictValues(mapValue(throttlePidValue, -45, 45, 400, 260)))

                rollServoValue = int(restrictValues(mapValue(rollPidValue)))
                print "roll servo value ", rollServoValue
                print "elevator Servo Value: ", elevatorServoValue

                servoDriver.setPWM(elevator_channel, 0, elevatorServoValue)
                servoDriver.setPWM(aileron_channel, 0, rollServoValue)
                servoDriver.setPWM(throttle_channel, 0, throttleServoValue)
                
                # Send telemetry through Xbee
                # Pitch, Roll, Latitude, Longitude, Altitude, Heading, elevatorServo, aileron servo, throttle servo
                data = str(pitch_angle) + ',' + str(roll_angle) + ',' + str(gpsd.fix.latitude) + ',' + 
                       str(gpsd.fix.longitude) + ',' + str(gpsd.fix.altitude) + ',' + str(heading) + ',' + 
                       str(elevatorServoValue) + ',' + str(rollServoValue) + ',' + str(throttleServoValue)
                GPIO.output(uart_toggle, True)
                time.sleep(0.01) # delay for 10 ms
                ser.write(data)
                
                #nextHeading = ser.read(31) # get target location from xbee. Needs figuring out because read will halt until characters are received
                # Need to figure out when there is a new target location
                
            elif(not auto_mode and cc_mode):
                #Make a PID for modulating Throttle...BILL!...neye the science guy.
                throttlePidValue = throttlePid.update(pitch_angle)
                throttleServoValue = int(restrictValues(mapValue(throttlePidValue, -45, 45, 400, 260)))
                servoDriver.setPWM(throttle_channel, 0, throttleServoValue)
                servoDriver.setPWM(rudder_channel, 0, 290) #fixed rudder
                #Make PID for the throttle
                
                # Send telemetry through Xbee
                # Pitch, Roll, Latitude, Longitude, Altitude, Heading, throttle servo, rudder servo
                data = str(pitch_angle) + ',' + str(roll_angle) + ',' + str(gpsd.fix.latitude) + ',' + 
                       str(gpsd.fix.longitude) + ',' + str(gpsd.fix.altitude) + ',' + str(heading) + ',' + 
                       str(290) + ',' + str(throttleServoValue)
                GPIO.output(uart_toggle, True)
                time.sleep(0.01) # delay for 10 ms
                ser.write(data)
            
            servoDriver.setPWM(pan_channel, 0, 290)
            servoDriver.setPWM(tilt_channel, 0, 250)
    except KeyboardInterrupt:
        #print "Killing program"
        ser.close()
        gpsp.running = False
        gpsp.join()
        time.sleep(1)