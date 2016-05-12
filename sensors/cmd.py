import sys
from Adafruit_LSM303 import Adafruit_LSM303
from Adafruit_PWM_Servo_Driver import PWM
servo = PWM(0x40)
servo.setPWMFreq(43)
while True:
    value = int(input())
    servo.setPWM(1, 0, value)
