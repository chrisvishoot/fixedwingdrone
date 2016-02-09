import time
import threading


print('Waiting')
time.sleep(2)
print('Done')


def gyro():
	while True:
		print('Gyro')
		time.sleep(.05)

def accel():
	while True:
		print('Accel')
		time.sleep(0.05)

get_gyro = threading.Thread(target=gyro)
get_accel = threading.Thread(target=accel)
get_gyro.start()
get_accel.start()


while True:
	
	time.sleep(1)

