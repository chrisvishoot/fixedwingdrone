#!/usr/bin/python

#
# FishPi - An autonomous drop in the ocean
#
# Simple test of GPS sensor
#

#import raspberrypi

from time import sleep
from GPS_serial import GPS_AdafruitSensor

if __name__ == "__main__":
    print "Testing GPS sensor (running 5x with 5s pause)..."

    print "Initialising..."
    gps_sensor = GPS_AdafruitSensor(debug=True, interface='/dev/ttyAMA0')
    #print raspberrypi.serial_bus()
    
    # heading
    print "Reading 1..."
    (fix, lat, lon, heading, speed, altitude, num_sat, timestamp, datestamp) = gps_sensor.read_sensor()
    print fix, lat, lon, heading, speed, altitude, num_sat, timestamp, datestamp
    sleep(5)
    
    print "Reading 2..."
    (fix, lat, lon, heading, speed, altitude, num_sat, timestamp, datestamp) = gps_sensor.read_sensor()
    print fix, lat, lon, heading, speed, altitude, num_sat, timestamp, datestamp
    sleep(5)

    print "Reading 3..."
    (fix, lat, lon, heading, speed, altitude, num_sat, timestamp, datestamp) = gps_sensor.read_sensor()
    print fix, lat, lon, heading, speed, altitude, num_sat, timestamp, datestamp
    sleep(5)
    
    print "Reading 4..."
    (fix, lat, lon, heading, speed, altitude, num_sat, timestamp, datestamp) = gps_sensor.read_sensor()
    print fix, lat, lon, heading, speed, altitude, num_sat, timestamp, datestamp
    sleep(5)
    
    print "Reading 5..."
    (fix, lat, lon, heading, speed, altitude, num_sat, timestamp, datestamp) = gps_sensor.read_sensor()
    print fix, lat, lon, heading, speed, altitude, num_sat, timestamp, datestamp
    sleep(5)
    
    print "Done."

    # raw values - not currently implemented any differently
    # will move more fields over when correctly reading sensor
    # gpsSensor.read_sensor_raw()
