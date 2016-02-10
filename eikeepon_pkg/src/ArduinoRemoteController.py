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
from eikeepon_pkg.msg import OmronMsg
from eikeepon_pkg.msg import ControllerMsg
from eikeepon_pkg.msg import ControllerBehaviorMsg

import serial
import math
from time import sleep
import sys
import logging


rospy.loginfo('$$$$$ Starting new running instance $$$$$')
rospy.loginfo('===========================================')

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

ard_serial = serial.Serial(ard_port, 9600, timeout=None) # no timeout
commandOut = None
print ard_serial

# --------------------------------------------------------------------
def main():
	# Initialize ROS node
	rospy.init_node('arduino_listener')
	
	# Iinitialize serial port to arduino
	try:
		sleep(1)
		commandOut =  'p' + str(90) \
				+ 't' + str(90) \
				+ 'r' + str(90) \
				+ 'b' + '0' 
		sendToArduino(commandOut)
	except Exception as e:
		rospy.logerr('(ARD_PY): Reading failed from arduino')
		rospy.logerr(e)

	# Subscribe to the Controller's stream
	rospy.Subscriber('controller_data', ControllerMsg, callback, queue_size=1)
	
	# Subscribe to the Behavior Controller's stream
	rospy.Subscriber('EIcontroller_data', ControllerBehaviorMsg, EIcallback, queue_size=1)
	rospy.spin()

#---------------------------------------------------------------------
def EIcallback(data):
	
	try:
		global launchID
		
		if data.ID != launchID:
			rospy.logdebug("ignoring message with id " + str(data.ID))
			return
			
		rospy.loginfo('received from EI controller action ' + str(data.action) + 'duration ' + str(data.duration))
		if data.action == 'idle':
			commandOut = 'ai'
			#commandOut = 'i0' + 'b' + '0'
		elif data.action == 'speak':
			commandOut = 'as'
			#rospy.loginfo("data duration " + str(data.duration))
			#commandOut = 's' + str((int)(data.duration)) + '\0'
		elif data.action == 'bounce':
			commandOut = 'ab'
		elif data.action == 'reset':
			commandOut = 'p90t90r90b0'
		sendToArduino(commandOut)
		rospy.loginfo("Sent to arduino " + commandOut)
		
	except Exception as e:
		rospy.logerr ("(ARD_PY) Sending to Arduino failed. Reopening serial connection.")
		rospy.logerr (e)
		ard_serial = serial.Serial(ard_port, 9600, timeout=None) # no timeout
		# Iterate through available ports, try connecting to each one
		# for port in list_ports.grep("ACM*"):
		# 	try:
		# 		ard_port = port[0]
		# 		print "trying to connect to port " + str(ard_port)
		# 		ard_serial = serial.Serial(ard_port, 9600, timeout=None)
		# 		break
		# 	except:# serial.SerialException:
		# 		print "failed to connect to port " + str(ard_port)
		# 		continue
				
		#ard_serial = serial.Serial(ard_port, 9600, timeout=None)
	print ''
	
# --------------------------------------------------------------------
def callback(data):
	try:
		global last_pan, ard_port, ard_serial, port_name
		printScreenLog('(ARD_PY): Received from controller ' + 'p:' + str(data.pan) + ', t:' 
				+ str(data.tilt) + ', r:' + str(data.roll))
			
		new_pan = last_pan + data.pan
		if new_pan < 0: 
			new_pan = 0
		elif new_pan > 180:
			new_pan = 180
		printScreenLog('(ARD_PY): Current pan: %s, Target: %s' % (last_pan,new_pan))
		commandOut =  'p' + str((int)(new_pan)) + \
					  't' + str((int)(data.tilt)) + \
					  'r' + str((int)(data.roll)) + 'b' + '0' 
		printScreenLog ('(ARD_PY): last_pan:%s, new_pan:%s' % (last_pan,new_pan))
		sendToArduino(commandOut)
	except Exception as e:
		rospy.logerr ("(ARD_PY) Sending to Arduino failed. Reopening serial connection.")
		rospy.logerr (e)
		if port_name == "ACM0": # switch ports
			port_name = "ACM1"
		elif port_name == "ACM1":
			port_name = "ACM0"
		ard_port = "/dev/tty" + port_name
		ard_serial = serial.Serial(ard_port, 9600, timeout=None)
	print ''

# --------------------------------------------------------------------
def sendToArduino (commandOut): # reads msgs from arduino, if any
	global last_pan
	global ard_serial
	ard_serial.flushOutput();	# clear the output buffer in case of backlog
	printScreenLog('(ARD_PY) Sending ---' + commandOut);
	ard_serial.write(commandOut)
			
	# while ard_serial.inWaiting() == 0:
	# 	pass
	# 	#print "waiting?"
	# while ard_serial.inWaiting() != 0:
	# 	msg = ard_serial.readline()
	# 	tt = '(ARDUINO): ' + msg
	# 	pan_pos = msg.find('pan:')
	# 	if pan_pos != -1:
	# 		pan_val_end = msg.find(',', pan_pos)
	# 		pan_value = int(msg[pan_pos+4:pan_val_end])
	# 		last_pan = pan_value
	# 	printScreenLog(tt)

# --------------------------------------------------------------------
def printScreenLog (string):
	string = string.strip() + '\n' 
	sys.stdout.write(string)
	rospy.logdebug(string.strip())

# --------------------------------------------------------------------
if __name__ == '__main__':
	main()
