import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
GPIO.setup(12,GPIO.OUT)


print'Hello'
while True:
	GPIO.output(12,1)
	time.sleep(0.25)
	GPIO.output(12,0)
	time.sleep(0.25)
