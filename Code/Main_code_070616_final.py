# -*- coding: utf-8 -*-
"""
Created on Tue May 10 15:24:22 2016

@author: pc
"""

import socket, json, threading, serial, pynmea2, argparse
import RPi.GPIO as GPIO, sys, time
from math import *

import smbus
import math

x_offset = 0
y_offset = 0


dest_lat = '33.546789'
dest_lon = '73.133222'
action = ''
distance = 0
state = 1
curr_angle = 0
angle = 0
#front_distance = 0
#stop = Fasle


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

slowspeed = 20
fastspeed = 100

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
    global curr_angle
    while 1:
	time.sleep(0.2)
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

	curr_angle = math.degrees(bearing)
	#print "Bearing: ", curr_angle
	#print  "Bearing: ", math.degrees(bearing)



def GPS():
  global dest_lat, dest_lon, dest_distance
  #C1
  #lat1 = 33.546803
  #lon1 = 73.132379
  #coordinates2
  #lat2 =  33.546803
  #lon2 =  73.133979 
  try:
    GPS_port = serial.Serial("/dev/ttyAMA0", 9600, timeout=0.5)
  except Exception as e:
    print "GPS EXCEPTION:::",e
  
  while True:
#    lat1 = 33.546801
#    lon1 = 73.132379

#    lat2 = 33.546801
#    lon2 = 73.133979 
    try:
      data = GPS_port.readline()
      if data.find('GGA') > 0:
        msg = pynmea2.parse(data)
        #
	if len(msg.lat) > 0:
		#print "Timestamp: %s -- Lat: %s %s -- Lon: %s %s -- Altitude: %s %s" % (msg.timestamp,msg.lat,msg.lat_dir,msg.lon,msg.lon_dir,msg.altitude,msg.altitude_units)
		lat_str = str(msg.lat)
		print "orig lat: ", lat_str
		lat_one = lat_str[0:2]
		lat_two = lat_str[2:]
		lat1 = (float(lat_one) + float(lat_two)/60 )
		lat1 = round(lat1,6)
		#print lat1
		
		
		lng_str = str(msg.lon)
		lng_str = lng_str[1:]
		print "orig lng: ", lng_str
		lng_one = lng_str[0:2]
		lng_two = lng_str[2:]
		lon1 = (float(lng_one) + float(lng_two)/60)
		lon1 = round(lon1,6)

		print "lat2:" , dest_lat, " lon2: ", dest_lon
		print "lat:" , lat1, " lon: ", lon1,          
		
		lat2 = float(dest_lat)
		lon2 = float(dest_lon)

		
		Aaltitude = 2000
		Oppsite  = 20000
		
		#Haversine Formuala to find vertical angle and distance
		lon1, lat1,lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])
		
		dlon = lon2 - lon1
		dlat = lat2 - lat1
		a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
		c = 2 * atan2(sqrt(a), sqrt(1-a))
		Base = (6371 * c ) * 1000
		time.sleep(1)
		#print("Distance:", Base,"m")
		dest_distance = round(Base,2)
		print "   dest_distance: ", dest_distance, "m"
		   



	else:
	  #print "Timestamp: %s - Waiting for GPS to get lock..." % msg.timestamp
	  print "GPS not Locked. Waiting for GPS lock..."
	  #print "lat: " + str(lat2) + " and lon: " + str(lon2)
	  time.sleep(1)

    except Exception as e:
      print e


def Movement(adr,port):
  global s, front_distance, left_distace, right_distance, lat2,lon2, action
  global angle#, stop
  print "started"

  s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  s.bind((adr,port))
  while True:
	    (data, addr) = s.recvfrom(1024)
	    print "Got >>", str(data)
	    
	    json_data = json.loads(data)

	    if "lat" in data:
		lat2 = json_data.get("lat")
		lon2 = json_data.get("lon")
		angle = json_data.get("angle")
		print "Got Lat lon and angle"

	    elif "action" in data:
		action = json_data.get("action")
		
		print "action: ", action
	 	if action == "stop":
		   #stop = True
		   stopall()
	
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
    global front_distance, state
	
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

        #print "front:",distance,"cm", state



def UV_right_range():
    TRIG = 24
    ECHO = 23
    
    global rigdht_distance
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



def set_head(my_angle):
    global curr_angle
    while curr_angle < my_angle-10 or curr_angle > my_angle+10 and action != "stop":
       #print "setlling angle >", curr_angle, " - ", my_angle	
       if my_angle > curr_angle:
	  p.ChangeDutyCycle(4)
	  q.ChangeDutyCycle(0)
	  a.ChangeDutyCycle(0)
 	  b.ChangeDutyCycle(50)
       elif my_angle < curr_angle:
	  p.ChangeDutyCycle(0)
	  q.ChangeDutyCycle(50)
	  a.ChangeDutyCycle(4)
  	  b.ChangeDutyCycle(0)
    stopall()
    return "done"


def auto_movement():
    global front_sensor
    global front_distance, left_distace, right_distance
    global lat2, angle, lon2, distance, curr_angle
    global state, action
    while 1:
	#    #print "auto started"
	    time.sleep(1)
	#    print "action: ", action
	#    print "state: ", state
	#    print "distance: ", distance
	#    print "curr_angle: ", curr_angle
	#    print "lon2: ", lon2
        #    print "lat2: ", lat2
	#    print "front Mov: ", front_sensor.range
	#    print "front Mov: ", front_distance, " " , left_distance, " " , right_distance 
    #while 1:
    #  if action == "start":

    #    if lat2 != '33.546111' and lon2 != '73.133222':
	#        if state == 1:
	#	   print "curr_angle: ", curr_angle	
	#           setting_head=set_head(angle)
	#	   if setting_head == "done":
	#	     print "done setting angle"
	 #	     state = 2
	      
	 #       if state == 2:
	 #          if front_distance > 70:
#		      forwards()
#		   else:
#		     print('obstacle STOPPING')
#		     stopall()
#		     time.sleep(3)
#		     state = 3
#	
#	        if state == 3:
#		   if left_distance > right_distance:
#	              turnleft()
#		      print('left')
#		      time.sleep(0.7)
#		      print('forwards')
#		      forwards()
#		      time.sleep(2)
#		      stopall()
#		      state = 1	
#		   elif right_distance > left_distance:
#	              turnright()
#		      print('right')
#		      time.sleep(0.7)
#		      print('forwards')
#		      forwards()
#		      time.sleep(2)
#		      stopall()
#		      state = 1	
#	
#	        if state != 4:
#	           if curr_angle < angle-10 or curr_angle > angle+10:
#		      state = 1
#	
#	        if distance < 0.00:
#		   state = 4
#	
#	        if state == 4:
 # 	   	   print "destination reached"
  #      else:
#	     print "please select any destination location"	  
 #     elif action == "stop":
#	   #print "stopping"
#	   state = 1
#	   stopall()
#


def main():
   global front_sensor
   my_args = argparse.ArgumentParser()
   my_args.add_argument('adr', type=str, help="IP address of raspberrypi")
   my_args.add_argument('port', type=int, help="port")
   args = my_args.parse_args()
   #print (args.adr)
   #print (args.port)

   #front_sensor = range_sensor("front", 27,17)
   #front_sensor.start()

   #print '>>>>', front_sensor.range

   movement_thread = threading.Thread(target=Movement, args=(args.adr,args.port,))
   movement_thread.start()

   GPS_thread = threading.Thread(target=GPS)
   GPS_thread.start() 

   auto_movement_thread = threading.Thread(target=auto_movement)
   auto_movement_thread.start()

   Compass_thread = threading.Thread(target=Compass)
   Compass_thread.start()  

   UV_left_thread = threading.Thread(target=UV_left_range)
   UV_left_thread.start()
    
   UV_front_thread = threading.Thread(target=UV_front_range)
   UV_front_thread.start()

   UV_right_thread = threading.Thread(target=UV_right_range)
   UV_right_thread.start()

   #left_sensor = range_sensor("left", 18,4)
   #left_sensor.start()

   #right_sensor = range_sensor("right", 23,24)
   #right_sensor.start()

if __name__ == "__main__":
   main()

