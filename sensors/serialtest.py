import serial
import time

delay = 1.0
com = serial.Serial('/dev/ttyAMA0', 9600)
#com.open()

while True:
    print com.readline()
    time.sleep(delay)
