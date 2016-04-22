#!/usr/bin/python

from Adafruit_PWM_Servo_Driver import PWM
import time

# ===========================================================================
# Example Code
# ===========================================================================

# Initialise the PWM device using the default address
pwm = PWM(0x40)
# Note if you'd like more debug output you can instead run:
#pwm = PWM(0x40, debug=True)

servoMin = 195  # Min pulse length out of 4096, 1.2 ms / 21 ms
servoMax = 410  # Max pulse length out of 4096, 1.9 ms / 21 ms


pwm.setPWMFreq(43)                        # Set frequency to 47 Hz, 21 ms
while (True):
  # Change speed of continuous servo on channel O
  print "Set servo 2 to", servoMin
  pwm.setPWM(2, 0, servoMin)
  pwm.setPWM(4, 0, servoMin)
  time.sleep(2)
  print "Set servo 2 to", servoMax
  pwm.setPWM(2, 0, servoMax)
  pwm.setPWM(4, 0, servoMax)
  time.sleep(2)



