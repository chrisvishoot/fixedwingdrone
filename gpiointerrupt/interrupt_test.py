import RPi.GPIO as GPIO
import time
from datetime import datetime

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(12,GPIO.OUT)
GPIO.setup(16,GPIO.OUT)
GPIO.setup(20,GPIO.OUT)

GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

p12 = GPIO.PWM(12,50)
p16 = GPIO.PWM(16,40)
p20 = GPIO.PWM(20,20)
p12.start(1)
p16.start(1)
p20.start(1)
p12.ChangeDutyCycle(20)
p16.ChangeDutyCycle(30)
p20.ChangeDutyCycle(50)

s0 = datetime.now().microsecond
s2 = datetime.now().microsecond
s1 = datetime.now().microsecond
st1 = 0
st2 = 0
st3 = 0
count = 0



def bttn_one(channel):
	if GPIO.input(17):
		global s0
		s0  = datetime.now().microsecond
	else:
		global st0 
		st0 = datetime.now().microsecond - s0
		global count
		count = count + 1
def bttn_two(channel):
	if GPIO.input(22):
		global s1
		s1 = datetime.now().microsecond
	else:
		global st1
		st1 = (datetime.now().microsecond - s1)
		global count
		count = count + 1
def bttn_three(channel):
	if GPIO.input(27):
		global s2
		s2 = datetime.now().microsecond
	else:
		global st2 
		st2 = (datetime.now().microsecond - s2)
		global count
		count = count + 1


GPIO.add_event_detect(17, GPIO.BOTH)
GPIO.add_event_detect(22, GPIO.BOTH)
GPIO.add_event_detect(27, GPIO.BOTH)
GPIO.add_event_callback(17, bttn_one)
GPIO.add_event_callback(22, bttn_two)
GPIO.add_event_callback(27, bttn_three)


start_time = datetime.now().second
time.sleep(2)

#while (datetime.now().second - start_time) < 2:
#	GPIO.wait_for_edge(17,GPIO.RISING)
#	s0 = datetime.now().microsecond
#	GPIO.wait_for_edge(17,GPIO.FALLING)
#	st0 = datetime.now().microsecond - s0 
#	GPIO.wait_for_edge(22,GPIO.RISING)
#	s1 = datetime.now().microsecond	
#	GPIO.wait_for_edge(22, GPIO.FALLING)
#	st1 = datetime.now().microsecond - s1
#	GPIO.wait_for_edge(27, GPIO.RISING)
#	s2 = datetime.now().microsecond
#	GPIO.wait_for_edge(27, GPIO.FALLING)
#	st2 = datetime.now().microsecond - s2
#	count = count + 1

print(count)
