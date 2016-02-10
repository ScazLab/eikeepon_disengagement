#! /usr/bin/env python
''' 
 * ArduinoRemoteController
 *
 * Subscriber to the ros stream from Controller.py
 * 
 * Communicates with the Arduino microcontroller to send commands 
 * to make the robot pan to the desired location (when a 'friend' has been
 * identified).
 * Once pan is stabilized, the Arduino is instructed to mimic the yaw, roll 
 * and tilt of the human target as much as possible.
 *
'''
	
import roslib; roslib.load_manifest('eikeepon_pkg')
import rospy

import serial
#from serial.tools import list_ports
import math
from time import sleep
import sys, cmd
import logging
import signal 

# --------------------------------------------------------------------
ard_serial = None	
port_name = "ACM0" # default port for the Arduino
launchID = 0 # default if no argument passed

argc = len(sys.argv)
if (argc > 1):
	port_name = sys.argv[1]
if (argc > 2):
	launchID = int(sys.argv[2])
	
ard_port = "/dev/tty" + port_name

ard_serial = serial.Serial(ard_port, 9600, timeout=None) # no timeout  timeout=None
commandOut = None
print ard_serial

import random

#idle behavior states
IDLE_FRONT = 0
IDLE_LEFT = 1
IDLE_RIGHT = 2
IDLE_UP = 3
IDLE_DOWN = 4
IDLE_BOP = 5
IDLE_MAX_STATES = 5

#different behavior "states" 
ANIM_IDLE = 0
ANIM_SPEAK = 1
ANIM_BOUNCE = 2
ANIM_SAD = 3
ANIM_HAPPY = 4

def idleLookFront():
	print "\nexecute idleLookFront"
	for x in range(0, 3):
		sleep(0.01)
		commandOut =  'p90' \
					+ 't90' \
					+ 'r90' \
					+ 'b0' +'\0' 
		printScreenLog("sending to arduino " + commandOut)
		sendToArduino(commandOut)
	last_roll = last_tilt = 90

def idleLookLeft():
	print "\nexecute idleLookLeft"
	for x in range(0, 3):
		sleep(0.01)
		commandOut =  'p00' \
					+ 't90' \
					+ 'r20' \
					+ 'b0' +'\0' 
		printScreenLog("sending to arduino " + commandOut)
		sendToArduino(commandOut)
	last_roll = -15
 
def idleLookRight():
	print "\nexecute idleLookRight"
	for x in range(0, 3):
		sleep(0.01)
		commandOut =  'p00' \
					+ 't90' \
					+ 'r160' \
					+ 'b10' +'\0' 
		printScreenLog("sending to arduino " + commandOut)
		sendToArduino(commandOut)
	last_roll = 130
	#idleLookFront()
 
def idleLookUp():
	print "\nexecute idleLookUp"
	for x in range(0, 3):
		sleep(0.01)
		commandOut =  'p90' \
					+ 't160' \
					+ 'r90' \
					+ 'b0' +'\0' 
		printScreenLog("sending to arduino " + commandOut)
		sendToArduino(commandOut)

def idleLookDown():
	print "\nexecute idleLookDown"
	for x in range(0, 3):
		sleep(0.01)
		commandOut =  'p90' \
					+ 't20' \
					+ 'r90' \
					+ 'b0' +'\0' 
		printScreenLog("sending to arduino " + commandOut)
		sendToArduino(commandOut)

def idleBop():
	print "\nexecute BOP"
	for x in range(0, 3):
		sleep(0.01)
		commandOut =  'p90' \
					+ 't0' \
					+ 'r90' \
					+ 'b10' +'\0' 
		printScreenLog("sending to arduino " + commandOut)
		sendToArduino(commandOut)

executeIdle = {IDLE_FRONT : idleLookFront,
                IDLE_LEFT : idleLookLeft,
                IDLE_RIGHT: idleLookRight,
                IDLE_UP: idleLookUp,
                IDLE_DOWN: idleLookDown,
                IDLE_BOP: idleBop
}

def speak():
	i = 0
	rollRight = 130
	rollLeft = 20

	i = -1;
	while 1:
		if i < 0:
			commandOut =  'p00' \
		 		+ 't90' \
		 		+ 'r' + str (rollRight) \
		 		+ 'b1' +'\0' 
			i = i*(-1)
		else:
			commandOut =  'p00' \
		 		+ 't90' \
		 		+ 'r' + str (rollLeft) \
		 		+ 'b1' +'\0' 
			i = i*(-1)
		printScreenLog("sending to arduino " + commandOut)
		sendToArduino(commandOut)
		sleep(0.1)

#--------------------------------------------------------

def signal_handler(signal, frame):
    print 'Closing the Arduino Port...'
    ard_serial.close()
    sys.exit()
    

# --------------------------------------------------------------------
def main():
	# Initialize ROS node
	rospy.init_node('arduino_listener')

	signal.signal(signal.SIGINT, signal_handler)
	# Iinitialize serial port to arduino
	try:
		sleep(1)
		commandOut =  'PA090S090TA000S200BA000E'
		sendToArduino(commandOut)
		
	except Exception as e:
		rospy.logerr('(ARD_PY): Reading failed from arduino')
		rospy.logerr(e)
		ard_serial.close()

	while 1:
		ard_serial.flushOutput();	# clear the output buffer in case of backlog
		try:
			line = sys.stdin.readline()
		except KeyboardInterrupt:
			break

		if not line:
			break

		#parse input
		print line

		sendToArduino(line)

	# Subscribe to the Controller's stream
	print 'before subscriber'
	rospy.Subscriber('controller_data', ControllerMsg, callback, queue_size=1)
	print 'after subscriber'
	rospy.spin()

def pollCurrentPosition():
	ard_serial.write('uu') # poll for update

	printScreenLog('(ARD_PY): Polling arduino...')
		
	while ard_serial.inWaiting() == 0: # waiting for input buffer
		pass
	while ard_serial.inWaiting() != 0: # something in input buffer
		msg = ard_serial.readline()
		tt = '(ARDUINO): ' + msg
		print tt + '\n'


# --------------------------------------------------------------------
def sendToArduino (commandOut): # reads msgs from arduino, if any
	global last_pan
	global ard_serial
	
	ard_serial.flushOutput();	# clear the output buffer in case of backlog
	ard_serial.write(commandOut)


# --------------------------------------------------------------------
def printScreenLog (string):
	string = string.strip() + '\n' 
	sys.stdout.write(string)
	rospy.logdebug(string.strip())

# --------------------------------------------------------------------
if __name__ == '__main__':
	main()
