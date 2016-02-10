#! /usr/bin/env python

import roslib; roslib.load_manifest('eikeepon_pkg')
import rospy
from eikeepon_pkg.msg import OmronMsg
from eikeepon_pkg.msg import ControllerMsg
from eikeepon_pkg.msg import ControllerBehaviorMsg

import thread
import threading
import socket
import sys
import string
import signal 
import pyaudio
import wave
from random import choice
import math
import random

import thread
import threading
import socket
import sys
import string
import signal 

import pyaudio
import wave
import sys
from random import choice
import time
import datetime

from ctypes import *
from contextlib import contextmanager




HOST = ''   # Symbolic name, meaning all available interfaces
PORT = 4510 # Arbitrary non-privileged port
BUFSIZ = 1024

hint = 'audio/AnnaEncourage.wav'

from eikeepon_pkg.msg import ControllerBehaviorMsg
from time import sleep

def setRobot(state, dur):
	message = ControllerBehaviorMsg()
	message.action = state
	message.duration = dur
	pub.publish(message)

def say(soundfile):

	print 'playing sound ' + soundfile
	wf = wave.open(soundfile, 'rb')
	p = pyaudio.PyAudio()
	stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
		channels=wf.getnchannels(),
		rate=wf.getframerate(),
		output=True)
	data = 	wf.readframes(8)
	while data != '':
		stream.write(data)
		data = wf.readframes(8)
	stream.stop_stream()
	stream.close()
	p.terminate()

def socketHandler(clientsock,addr):	
	
	while 1:	# waiting for messages
		msg = clientsock.recv(BUFSIZ)
		#print 'data from socket ' + data

		#print 'receive message: ' + msg  #print message
		
		if (msg == '1'):
			print 'confused!'
			setRobot("speak", 0)
			say(hint)

		setRobot("idle", 0)

	clientsock.close()


def signal_handler(signal, frame):
    print 'Closing the socket and exiting...'
    try:
    	s.close()
    except socket.error as msg:
    	print 'Closing scoket failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
    	sys.exit()
    sys.exit(0)



def main():
	print 'HEY THERE'
	global pub
	global s #socket
	
	print 'on main'

	# Initialize ROS node
	rospy.init_node('controller')

	# Create publisher to send out ControllerMsg messages
	pub = rospy.Publisher('EIcontroller_data', ControllerBehaviorMsg)
	
	rospy.loginfo("Controller node inititalized.")
	rospy.sleep(2.0) #important!
	
	#Initialize socket to connect with haldheld device
	signal.signal(signal.SIGINT, signal_handler)
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	print 'Socket created' 
	#Bind socket to local host and port
	try:
		s.bind((HOST, PORT))
	except socket.error as msg:
		print 'Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
		sys.exit()
    
	print 'Socket bind complete'
     
	#Start listening on socket
	s.listen(10)
	print 'Socket now listening'
 
	#now keep talking with the client
	while 1:
		#wait to accept a connection - blocking call
		print 'waiting for connection'
		conn, addr = s.accept()
		print 'Connected with ' + addr[0] + ':' + str(addr[1])
		#thread.start_new_thread(interactionScript,(conn,1))
		thread.start_new_thread(socketHandler, (conn, addr))

	while not rospy.is_shutdown():
		rospy.sleep(2.0)
		#interactionScript(pub,2)

if __name__ == '__main__':
	main()
		
