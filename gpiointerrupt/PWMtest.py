import RPIO
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
GPIO.setup(12,GPIO.OUT)
p = GPIO.PWM(12, 50)
p.start(1)
perc = 1
print'Hello'
while True:
	
	time.sleep(0.01)
	p.ChangeDutyCycle(perc)
	if perc > 99:
		perc = 1
	perc = perc + 1

