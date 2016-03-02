#! /usr/bin/python
# Written by Dan Mandle http://dan.mandle.me September 2012
# License: GPL 2.0
 
import os
from gps import *
from time import *
import time
import threading
import sqlite3
 
gpsd = None #seting the global variable
 
os.system('clear') #clear the terminal (optional)
 
class GpsPoller(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
    global gpsd #bring it in scope
    gpsd = GPS(mode=WATCH_ENABLE) #starting the stream of info
    self.current_value = None
    self.running = True #setting the thread running to true
 
  def run(self):
    global gpsd
    while gpsp.running:
      gpsd.next() #this will continue to loop and grab EACH set of gpsd info to clear the buffer

if __name__ == '__main__':
  gpsp = GpsPoller() # create the thread
  conn = sqlite3.connect('gpsdata.db')
  c = conn.cursor()
  try:
    gpsp.start() # start it up
    while True:
      #It may take a second or two to get good data
      #print gpsd.fix.latitude,', ',gpsd.fix.longitude,'  Time: ',gpsd.utc
 
      os.system('clear')
      
      data = (gpsd.utc, gpsd.fix.latitude, gpsd.fix.longitude, gpsd.fix.altitude, gpsd.fix.track, gpsd.fix.speed, gpsd.fix.climb, gpsd.fix.eps, gpsd.fix.epx, gpsd.fix.epv, gpsd.fix.ept, gpsd.fix.mode, len(gpsd.satellites))
      print(data)
      try:
        c.execute('INSERT INTO gps VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?)', data)
      except (sqlite3.IntegrityError):
        pass
      conn.commit()

      print()
      print(' GPS reading')
      print('----------------------------------------')
      print('time utc    ' , gpsd.utc)
      print('latitude    ' , gpsd.fix.latitude)
      print('longitude   ' , gpsd.fix.longitude)
      print('altitude (m)' , gpsd.fix.altitude)
      print('track       ' , gpsd.fix.track)
      print('speed (m/s) ' , gpsd.fix.speed)
      print('climb       ' , gpsd.fix.climb)
      print('eps         ' , gpsd.fix.eps)
      print('epx         ' , gpsd.fix.epx)
      print('epv         ' , gpsd.fix.epv)
      print('ept         ' , gpsd.fix.ept)
      print('mode        ' , gpsd.fix.mode)
      print('sats        ' , len(gpsd.satellites))
 
      time.sleep(1) #set to whatever
 
  except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
    conn.close()
    print("\nKilling Thread...")
    gpsp.running = False
    gpsp.join() # wait for the thread to finish what it's doing
  print("Done.\nExiting.")

