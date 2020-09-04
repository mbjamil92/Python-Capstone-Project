# -*- coding: utf-8 -*-
"""
Created on Tue May 10 15:24:22 2016

@author: pc
"""

import socket, json, threading, serial, pynmea2
import RPi.GPIO as GPIO, sys, time
from math import *

import smbus
import math

x_offset = 24
y_offset = 177

#use physical pin numbering
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

#use pwm on inputs so motors don't go too fast
# Pins 19, 26 Right Motor
# Pins 20, 21 Left Motor
GPIO.setup(19, GPIO.OUT)
p=GPIO.PWM(19, 20)
p.start(0)
GPIO.setup(26, GPIO.OUT)
q=GPIO.PWM(26, 20)
q.start(0)
GPIO.setup(20, GPIO.OUT)
a=GPIO.PWM(20,20)
a.start(0)
GPIO.setup(21, GPIO.OUT)
b=GPIO.PWM(21,20)
b.start(0)

#slowspeed = 20
#fastspeed = 100

slowspeed = 10
fastspeed = 75

LED1 = 22
LED2 = 18
LED3 = 11
LED4 = 07
GPIO.setup(LED1, GPIO.OUT)
GPIO.setup(LED2, GPIO.OUT)
GPIO.setup(LED3, GPIO.OUT)
GPIO.setup(LED4, GPIO.OUT)

def reverse():
  p.ChangeDutyCycle(fastspeed)
  q.ChangeDutyCycle(0)
  a.ChangeDutyCycle(fastspeed)
  b.ChangeDutyCycle(0)
  #setLEDs(1, 0, 0, 1)
  #print('straight')

def forwards():
  p.ChangeDutyCycle(0)
  q.ChangeDutyCycle(fastspeed)
  a.ChangeDutyCycle(0)
  b.ChangeDutyCycle(fastspeed)
  #setLEDs(0, 1, 1, 0)
  #print('straight')

def turnright():
  p.ChangeDutyCycle(slowspeed)
  q.ChangeDutyCycle(0)
  a.ChangeDutyCycle(0)
  b.ChangeDutyCycle(fastspeed)
  #setLEDs(0, 0, 1, 1)
  #print('left')

def turnleft():
  p.ChangeDutyCycle(0)
  q.ChangeDutyCycle(fastspeed)
  a.ChangeDutyCycle(slowspeed)
  b.ChangeDutyCycle(0)
  #setLEDs(1, 1, 0, 0)
  #print('right')

def stopall():
  p.ChangeDutyCycle(0)
  q.ChangeDutyCycle(0)
  a.ChangeDutyCycle(0)
  b.ChangeDutyCycle(0)
  #setLEDs(1, 1, 1, 1)
  #print('stop')


front_distance = 0
right_distance = 0
left_distance = 0

###########################i2c

bus = smbus.SMBus(1)
address = 0x1e


def read_byte(adr):
    return bus.read_byte_data(address, adr)

def read_word(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def write_byte(adr, value):
    bus.write_byte_data(address, adr, value)


def Compass():
    while 1:
	time.sleep(0.1)
	write_byte(0, 0b01110000) # Set to 8 samples @ 15Hz
	write_byte(1, 0b00100000) # 1.3 gain LSb / Gauss 1090 (default)
	write_byte(2, 0b00000000) # Continuous sampling
	global x_offset, y_offset
	scale = 0.92
	#x_offset = -100
	#y_offset = 150
	x_out = (read_word_2c(3) - x_offset) * scale
	y_out = (read_word_2c(7) - y_offset) * scale
	z_out = read_word_2c(5) * scale
	
	bearing  = math.atan2(y_out, x_out) 
	if (bearing < 0):
	    bearing += 2 * math.pi
	
	print  "Bearing: ", math.degrees(bearing)


def Compass_callibrate():
	minx = 0
	maxx = 0
	miny = 0
	maxy = 0
	
	write_byte(0, 0b01110000) # Set to 8 samples @ 15Hz
	write_byte(1, 0b00100000) # 1.3 gain LSb / Gauss 1090 (default)
	write_byte(2, 0b00000000) # Continuous sampling
	
	scale = 0.92
	
	for i in range(0,500):
	    x_out = read_word_2c(3)
	    y_out = read_word_2c(7)
	    z_out = read_word_2c(5)
	    
	    
	    if x_out < minx:
	        minx=x_out
	    
	    if y_out < miny:
	        miny=y_out
	    
	    if x_out > maxx:
	        maxx=x_out
	    
	    if y_out > maxy:
	        maxy=y_out
	    
	    print x_out, y_out, (x_out * scale), (y_out * scale), i
	    time.sleep(0.1)
	
	print "minx: ", minx
	print "miny: ", miny
	print "maxx: ", maxx
	print "maxy: ", maxy
	print "x offset: ", (maxx + minx) / 2
	print "y offset: ", (maxy + miny) / 2


def GPS():

  lat1 = 75#53.32055
  lon1 = 33.5#-1.72972
  #coordinates2
  lat2 =  76#53.31861
  lon2 =  32.5#-1.69972  
  try:
    GPS_port = serial.Serial("/dev/ttyAMA0", 9600, timeout=0.5)
  except Exception as e:
    print "GPS EXCEPTION:::",e
  
  while True:
    data = GPS_port.readline()
    if data.find('GGA') > 0:
        msg = pynmea2.parse(data)
        #print "Timestamp: %s -- Lat: %s %s -- Lon: %s %s -- Altitude: %s %s" % (msg.timestamp,msg.lat,msg.lat_dir,msg.lon,msg.lon_dir,msg.altitude,msg.altitude_units)

        Aaltitude = 2000
        Oppsite  = 20000

        #Haversine Formuala to find vertical angle and distance
        lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
        c = 2 * atan2(sqrt(a), sqrt(1-a))
        Base = 6371 * c

        #Horisontal Bearing
        def calcBearing(lat1, lon1, lat2, lon2):
            dLon = lon2 - lon1
            y = sin(dLon) * cos(lat2)
            x = cos(lat1) * sin(lat2) \
                - sin(lat1) * cos(lat2) * cos(dLon)
            return atan2(y, x)

        Bearing = calcBearing(lat1, lon1, lat2, lon2)
        Bearing = degrees(Bearing)

        Base2 = Base * 1000
        distance = Base * 2 + Oppsite * 2 / 2
        Caltitude = Oppsite - Aaltitude


        #Convertion from radians to decimals
        a = Oppsite/Base
        b = atan(a)
        c = degrees(b)


        #Convert meters into Kilometers
        distance = distance / 1000


        #Output the data
        #print("---------------------------------------")
        #print(":::::Auto Aim Directional Anntenna:::::")
        #print("---------------------------------------")
        #print("Horizontial Distance:", Base,"km")
        #print("   Vertical Distance:", distance,"km")
        #print("    Vertical Bearing:",c)
        #print(" Horizontial Bearing:",Bearing)
        #print("---------------------------------------")

def Movement():
  global s, front_distance, left_distace, right_distance
  print "started"

  s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  try:
        s.bind(("192.168.1.3",8062))

	while True:
	    (data, addr) = s.recvfrom(1024)
	    #print "Got >>", str(data)
	    
	    json_data = json.loads(data)
	    FB = json_data.get("FB")
	    
	    LR = json_data.get("LR")
	
	    #print FB + " " + LR + str(len(FB)) + str(len(LR))
	    if FB == "F":
	      #print front_distance
	      if front_distance > 40:
	         forwards()
	      else:
	         stopall()
	
	    elif FB == "B":
	        reverse()
	        pass
	
	    elif LR == "L":
	        turnleft()
	        pass
	
	    elif LR == "R":
	        turnright()
	        pass
	
	    elif LR == "S" or FB == "S":
	        #print 'stop'
	        stopall()
  except Exception as e:
    print e
  finally:
    s.close()

  

def UV_left_range():
    TRIG = 4 
    ECHO = 18
    global left_distance
    #print "Distance Measurement In Progress"

    GPIO.setup(TRIG,GPIO.OUT)
    GPIO.setup(ECHO,GPIO.IN)
    while 1:
        GPIO.output(TRIG, False)
        #print "Waiting For Sensor To Settle"
        time.sleep(2)

        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        while GPIO.input(ECHO)==0:
          pulse_start = time.time()

        while GPIO.input(ECHO)==1:
          pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start

        distance = pulse_duration * 17150

        distance = round(distance, 2)
	left_distance = distance
        #print "left:",distance,"cm"
        
def UV_front_range():
    TRIG = 17
    ECHO = 27
    global front_distance
	
    #print "Distance Measurement In Progress"

    GPIO.setup(TRIG,GPIO.OUT)
    GPIO.setup(ECHO,GPIO.IN)
    while 1:
        GPIO.output(TRIG, False)
        #print "Waiting For Sensor To Settle"
        time.sleep(2)

        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        while GPIO.input(ECHO)==0:
          pulse_start = time.time()

        while GPIO.input(ECHO)==1:
          pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start

        distance = pulse_duration * 17150

        distance = round(distance, 2)

	front_distance = distance

        print "front:",distance,"cm"



def UV_right_range():
    TRIG = 24
    ECHO = 23
    
    global right_distance
    #print "Distance Measurement In Progress"

    GPIO.setup(TRIG,GPIO.OUT)
    GPIO.setup(ECHO,GPIO.IN)
    while 1:
        GPIO.output(TRIG, False)
        #print "Waiting For Sensor To Settle"
        time.sleep(2)

        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        while GPIO.input(ECHO)==0:
          pulse_start = time.time()

        while GPIO.input(ECHO)==1:
          pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start

        distance = pulse_duration * 17150

        distance = round(distance, 2)

	right_distance = distance

        #print "right:",distance,"cm"


def main():

   movement_thread = threading.Thread(target=Movement)
   movement_thread.start()

   GPS_thread = threading.Thread(target=GPS)
   #GPS_thread.start() 

   Compass_thread = threading.Thread(target=Compass)
   Compass_thread.start()  

   Compass_callibrate_T =  threading.Thread(target=Compass_callibrate)
   #Compass_callibrate_T.start()  

   UV_left_thread = threading.Thread(target=UV_left_range)
   #UV_left_thread.start()
    
   UV_front_thread = threading.Thread(target=UV_front_range)
   #UV_front_thread.start()

   UV_right_thread = threading.Thread(target=UV_right_range)
   #UV_right_thread.start()


if __name__ == "__main__":
   try:
      main()
   except KeyboardInterrupt:
      print "ended"
      sys.exit()
      #pass

