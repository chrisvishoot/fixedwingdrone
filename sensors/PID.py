import numpy as np
import time
import sys
from L3GD20 import L3GD20
from Adafruit_LSM303 import Adafruit_LSM303
from Adafruit_PWM_Servo_Driver import PWM
import math

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

def findMapping(pid, start1, end1, start2, end2):
    val1 = pid.update(start1)
    val2 = pid.update(end1)
    
    x1 = start2 # Lower bound for mapped range
    x2 = end2 # Upper bound for mapped range

    print val1, val2, x1, x2

    m = (x1 - x2) / (val1 - val2)

    b = x1 - (m * val1)
    
    print "b, m", b, m

    return b, m

def mapValue(value, b, m):
    return (value * m) + b
    
def getPitchYawRoll(Gyro, Accel):
    conv = 180 / math.pi

    (wx, wy, wz) = Gyro.Get_CalOut_Value()
    [[ax, ay, az],[magx, magy, magz, orientation]] = Accel.read()
    heading = (math.atan2(magy,magx)* 180)/ math.pi
    roll_angle = math.atan2(ay,az)
    pitch_in = -ax/((ay*math.sin(roll_angle)) + (az*math.cos(roll_angle)))
    pitch_angle = math.atan(pitch_in)
    yaw_y = (magz*math.sin(roll_angle)) - (magy*math.cos(roll_angle))   
    yaw_x = (magx*math.cos(pitch_angle)) + (magy*math.sin(pitch_angle)*math.sin(roll_angle)) + (magz*math.sin(pitch_angle)*math.cos(roll_angle))
    yaw_angle = math.atan2(yaw_y,yaw_x)
    pitch_angle = round(pitch_angle * conv,2)
    roll_angle = round(roll_angle * conv,2)
    yaw_angle = round(yaw_angle * conv,2)
    
    return (pitch_angle, yaw_angle, roll_angle)

in_min = -45
in_max = 45
out_min = 410
out_max = 195

def mapValue2(value):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def restrictValues(value):
    if value < 195:
	return 195
    elif value > 410:
	return 410
    else:
	return value


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

    elevator = PWM(0x40)
    elevator.setPWMFreq(43)
    
    elevatorUp = 195
    elevatorDown = 410

    elevatorPid = PID()
    
    pitch_angle, yaw_angle, roll_angle = getPitchYawRoll(Gyro, Accel)
    
    elevatorPid.setPoint(pitch_angle)
    
    b, m = findMapping(elevatorPid, -90, 90, 410, 195)

    try:
        while True:
            pitch_angle, yaw_angle, roll_angle = getPitchYawRoll(Gyro, Accel)
            
            elevatorPidValue = elevatorPid.update(pitch_angle)

            print('Pitch: ' + str(pitch_angle) + '\tYaw: ' + str(yaw_angle) + '\tRoll: ' + str(roll_angle))
            print "PID Elevator: ", elevatorPidValue

            elevatorServoValue = int(restrictValues(mapValue2(elevatorPidValue)))
            print "Elevator Servo Value: ", elevatorServoValue
            
            elevator.setPWM(2, 0, elevatorServoValue)
            
    except KeyboardInterrupt:
        print "Killing program"
        time.sleep(1)
