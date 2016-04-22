import time 
from datetime import datetime 
from L3GD20 import L3GD20 
from Adafruit_LSM303 import Adafruit_LSM303 
import math 

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

while True:
    (wx, wy, wz) = Gyro.Get_CalOut_Value()
    [[ax, ay, az],[magx, magy, magz, orientation]] = accel.read()
    heading = (math.atan2(magy,magx)* 180)/3.141592
    #print('wx: ' + str(wx) + ', wy: ' + str(wy) + ', wz: ' + str(wz) + 
    #', magx: ' + str(magx) + ', magy: ' + str(magy) + ', magz: ' + 
    #str(magz)) print('magx: ' + str(magx) + ', magy: ' + str(magy) + ', 
    #magz: ' + str(magz)) print(str(heading))
    time.sleep(0.05)
    roll_angle = math.atan2(ay,az)
    pitch_in = -ax/((ay*math.sin(roll_angle)) + (az*math.cos(roll_angle)))
    pitch_angle = math.atan(pitch_in)
    yaw_y = (magz*math.sin(roll_angle)) - (magy*math.cos(roll_angle))
    yaw_x = (magx*math.cos(pitch_angle)) + (magy*math.sin(pitch_angle)*math.sin(roll_angle)) + (magz*math.sin(pitch_angle)*math.cos(roll_angle))
    yaw_angle = math.atan2(yaw_y,yaw_x)
    pitch_angle = round(pitch_angle * conv,2)
    roll_angle = round(roll_angle * conv,2)
    yaw_angle = round(yaw_angle * conv,2)
    
    print('Pitch: ' + str(pitch_angle) + '\t Roll: ' + str(roll_angle) + '\t Yaw: ' + str(yaw_angle))
