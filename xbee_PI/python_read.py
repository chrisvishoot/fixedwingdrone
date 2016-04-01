import serial
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
rtsPin = 23
GPIO.setup(rtsPin, GPIO.OUT)
#  do not send to serial
GPIO.output(rtsPin, GPIO.LOW)

ser = serial.Serial('/dev/ttyAMA0', baudrate = 9600)
#to_send = "Hello  from Raspberry Pi!"
#print("Sending the message {0}".format(to_send))
#ser.write("%s\n" % to_send)

sleepTime = 0


while True:
#	print("Program is suspended")
#	time.sleep(sleepTime)
	print("Program is no longer suspended")
#	GPIO.output(rtsPin, GPIO.LOW)
	sleepTime = 0
	incoming = ser.readline().strip()
	print("Received the message {0}".format(incoming))
	ser.write("RPI received the message {0}".format(incoming))

