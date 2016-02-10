#! /usr/bin/env python
import roslib; roslib.load_manifest('eikeepon_pkg')
import rospy
from eikeepon_pkg.msg import ControllerBehaviorMsg, DisengagementPrediction

import serial
import math
from time import sleep
import sys
import random
import string
import signal 

from timeit import default_timer as timer

#idle behavior states
IDLE_N_STATES = 7
IDLE_FRONT = 0
IDLE_UP = 1
IDLE_DOWN = 2
IDLE_ROLL_LEFT = 3
IDLE_ROLL_RIGHT = 4
IDLE_BOPUP = 5
IDLE_BOPDOWN = 6
#IDLE_BOP = 5

#different behavior "states" 
ANIM_IDLE = 0
ANIM_SPEAK = 1
ANIM_BOUNCE = 2
#ANIM_SAD = 3
#ANIM_HAPPY = 4

#limits of motor commands
MIN = 0
MAX = 180
MIDDLE = 90

launchID = 0 # default if no argument passed
ard_serial = None	
port_name = "ACM0" # default port for the Arduino

#the higher these values the 'slower' the robots
TIMEOUT_SPEAK = 0.5
TIMEOUT_BOP = 0.6
TIMEOUT_IDLE = 4

last_roll_time = 0
last_bop_time = 0
last_idle_time = 0

roll_direction = 1 # positive or negative
bop_direction = 1 # positive or negative

current_anim = ANIM_IDLE

def anim_callback(data):
	global current_anim, launchID, port_name

	try:
		if data.ID != launchID:
			rospy.logdebug("ignoring message with id " + str(data.ID))
			return
		if data.action == 'idle':
			current_anim = ANIM_IDLE
		elif data.action == 'speak':
			current_anim = ANIM_SPEAK
		elif data.action == 'bounce':
			current_anim = ANIM_BOUNCE
		elif data.action == 'reset':
			commandOut = pan_abs(MIDDLE) + tilt_abs(MIDDLE) + roll_abs(MIDDLE) + bop_abs(MIDDLE)
			sendToArduino(commandOut)
		elif data.action == 'pan':
			sendToArduino(pan_abs(data.duration))

	except Exception as e:
		rospy.logerr ("(ARD_PY) Sending to Arduino failed. Reopening serial connection.")
		rospy.logerr (e)
		ard_serial = serial.Serial(port_name, 9600, timeout=None) # no timeout

def idleLookFront():
	command = tilt_abs(MIDDLE) + roll_abs(MIDDLE) + bop_abs(MIDDLE)
	sendToArduino(command)

def idleBopUp():
	sendToArduino(bop_abs(random.randint(MAX-35, MAX-15)))

def idleBopDown():
	sendToArduino(bop_abs(random.randint(MIN+10, MIN + 35)))

def idleLookUp():
	sendToArduino(tilt_abs(random.randint(MAX-20, MAX)))

def idleLookDown():
	sendToArduino(tilt_abs(random.randint(MIN, MIN+20)))

def idleRollLeft():
	sendToArduino(roll_abs(random.randint(MIN, MIN+20)))

def idleRollRight():
	sendToArduino(roll_abs(random.randint(MAX-20, MAX)))

executeIdle = {IDLE_FRONT : idleLookFront,
               IDLE_ROLL_LEFT: idleRollLeft,
               IDLE_ROLL_RIGHT: idleRollRight,
               IDLE_UP: idleLookUp,
               IDLE_DOWN: idleLookDown,
               IDLE_BOPUP: idleBopUp,
               IDLE_BOPDOWN: idleBopDown
	}

def runIdleStep():
	global last_idle_time

	current_time = timer()

	if (current_time - last_idle_time) > TIMEOUT_IDLE:
		last_idle_time = timer()
		action = random.randint(0, IDLE_N_STATES - 1)  # Integer from 1 to IDLE_MAX_STATES, endpoints included
		executeIdle[action]()


def runSpeakStep():
	global roll_direction, last_roll_time

	current_time = timer()

	if (current_time - last_roll_time) > TIMEOUT_SPEAK:
		last_roll_time = timer()
		roll_direction = roll_direction * -1
		if roll_direction > 0:
			sendToArduino(roll_abs(random.randint(MAX - 30, MAX)))
		else:
			sendToArduino(roll_abs(random.randint(MIN, MIN + 30)))

def runBopStep():
	global bop_direction, last_bop_time, roll_direction, last_roll_time

	current_time = timer()

	if (current_time - last_bop_time) > TIMEOUT_BOP:
		last_bop_time = timer()
		bop_direction = bop_direction * -1

		if bop_direction > 0:
			sendToArduino(bop_abs(random.randint(MAX - 45, MAX-35)))
		else:
			sendToArduino(bop_abs(random.randint(MIN+20, MIN + 30)))

def runBounceStep():
	runSpeakStep()
	runBopStep()

def sendToArduino (commandOut): # reads msgs from arduino, if any
		ard_serial.flushOutput();	# clear the output buffer in case of backlog
		print commandOut + 'E'
		ard_serial.write(commandOut + 'E')	

'''
	Returns a pan absolute command in the form of PNNN; ex: PA090
'''
def pan_abs( value):
	pan = 'PA' + str(value).zfill(3) + 'S050'
	return pan

'''
	Returns a tilt absolute command in the form of TANNN; ex: PA090
'''
def tilt_abs( value):
	tilt = 'TA' +  str(value).zfill(3) + 'S090'
	return tilt

'''
	Returns a roll absolute command in the form of RANNN; ex: RA090
'''
def roll_abs(value):
	roll = 'RA' +  str(value).zfill(3) + 'S090'
	return roll

'''
	Returns a bop absolute command in the form of BANNN; ex: BA090
'''
def bop_abs( value):
	bop = 'BA' +  str(value).zfill(3) + 'S090'
	return bop

def signal_handler(signal, frame):
	print 'Closing the Arduino Port...'
	command = tilt_abs(MIDDLE) + roll_abs(MIDDLE) + bop_abs(MIDDLE)
	sendToArduino(command)
	ard_serial.close()
	sys.exit()

if __name__ == '__main__':
	global current_anim, launchID

	# Initialize ROS node
	rospy.init_node('keepon_animation')

	port_name = 'ACM0'
	launchID = 0

	argc = len(sys.argv)
	if (argc > 1):
		port_name = sys.argv[1]
	if (argc > 2):
		launchID = int(sys.argv[2])
	
	ard_port = "/dev/tty" + port_name

	ard_serial = serial.Serial(ard_port, 9600, timeout=None) # no timeout

	print 'launch ID: ' + str(launchID)
	print 'port ' + str(port_name)
	print 'ard serial ' + str(ard_serial)
	
	rospy.Subscriber('EIcontroller_data', ControllerBehaviorMsg, anim_callback, queue_size=1)

	#reset all motors to 90
	command = pan_abs(MIDDLE) + tilt_abs(MIDDLE) + roll_abs(MIDDLE) + bop_abs(MIDDLE)
	sendToArduino(command)

	#to allow closing the program using ctrl+C
	signal.signal(signal.SIGINT, signal_handler)

	while(1):
		#runBounceStep()
		runIdleStep()
		if current_anim == ANIM_IDLE:
			runIdleStep()
		elif current_anim == ANIM_SPEAK:
			runSpeakStep()
		elif current_anim == ANIM_BOUNCE:
			runBounceStep()

		#runIdleStep()
		#runSpeakStep()
		#just for debugging
	'''	
		try:
			line = sys.stdin.readline()
		except KeyboardInterrupt:
			break

		if 'i' in line:
			current_anim = ANIM_IDLE
		elif 's' in line:
			current_anim = ANIM_SPEAK
		elif 'b' in line:
			current_anim = ANIM_BOUNCE	


	#sleep(1)
	rospy.spin()
'''	