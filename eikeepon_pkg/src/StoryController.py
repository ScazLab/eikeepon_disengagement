#! /usr/bin/env python

import roslib; roslib.load_manifest('eikeepon_pkg')
import rospy
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
import json

from timeit import default_timer as timer
from std_msgs.msg import String #HERE

from ctypes import *
from contextlib import contextmanager
from eikeepon_pkg.msg import ControllerBehaviorMsg, LogMsg, RobotFeaturesMsg, DisengagementPrediction, LogDisengagementMsg

from time import sleep

HOST = ''   # Symbolic name, meaning all available interfaces
PORT = 4444 # Arbitrary non-privileged port
BUFSIZ = 1024
ROBOT_A_ID = 1
ROBOT_B_ID = 2

STATE_PLAYING = 0
STATE_WAITING_CHOICE = 1
STATE_CHOICE_MADE = 2
STATE_WAITING_START = 3
STATE_INTERRUP = 4

CHOICE = 0 #TESTING

PAUSE = 0.5 #constant pause value after each utterance (adds to the "pause" value in each action)

interactionStartTime = 0
#comment vfdb

#stuff for interruptionsd
MAX_INTERRUPTIONS = 2 # 2 #maximum number of interruptions PER SCENE
MIN_TIME = 30 # 30 #minimum time between interruptions
DECAY_FACTOR = 1 #decrease in disengagement per second
DIS_THRESHOLD = 15 #number of dis instances before the robots do something

dis_left = 0
dis_middle = 0
dis_right = 0

nr_interruptions = 0
last_interrupt_time = 0
last_message_time = 0

group_interrupt_sound_path_R1 = ["dis-strategies/D1-R1.wav", "dis-strategies/D2-R1.wav",
"dis-strategies/D4-R1.wav", "dis-strategies/D5-R1.wav", "dis-strategies/D6-R1.wav",
"dis-strategies/D7-R1.wav", "dis-strategies/D8-R1.wav"]

ind_interrupt_sound_path_R1 = ["dis-strategies/D16-R1.wav", "dis-strategies/D17-R1.wav", 
"dis-strategies/D19-R1.wav", "dis-strategies/D18-R1.wav", "dis-strategies/D15-R1.wav", 
"dis-strategies/D14-R1.wav"]

ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)
def py_error_handler(filename, line, function, err, fmt):
  pass

c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)

asound = cdll.LoadLibrary('libasound.so.2')
# Set error handler
asound.snd_lib_error_set_handler(c_error_handler)

def __del__(self):
	s.close()
	
def signal_handler(signal, frame):
    print 'Closing the socket and exiting...'
    try:
    	s.close()
    except socket.error as msg:
    	print 'Closing scoket failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
    	sys.exit()
    sys.exit(0)
    
def socketHandler(clientsock,addr):
	global interactionState
	global logfile
	global currentSceneName
	global interactionStartTime
	
	while 1:
		try:
			data = clientsock.recv(BUFSIZ)
		except socket.timeout:
			print 'timeout exception'
			data="NO RESPONSE"

		#print 'data from socket ' + data
		if not data or data == "NO RESPONSE":
			continue

		msg = data.rstrip()
		msg = data.lstrip()
		msg = data.strip()
		print 'receive message: ' + msg

		#set state to start over interaction
		if ('startover' in msg):	
			currentSceneName = "Intro"
			print "startover MSG: " + msg
			splitMsg = msg.split(':')
			print "AFTER SPLITTING: " + splitMsg[1]
			loadScript(splitMsg[1])
			if interactionStartTime == 0:
				interactionStartTime = rospy.get_rostime().secs
				nsecs_interactionStartTime = rospy.get_rostime().nsecs
			interactionState = STATE_PLAYING
	
		if (interactionState == STATE_WAITING_CHOICE):
			currentSceneName = msg
			interactionState = STATE_CHOICE_MADE
			logAction("state", "STATE_CHOICE_MADE", 0)
			logAction("choice", msg, 0)
			setRobot(1, "pan",90)
			setRobot(2, "pan",90)

	if(data == "quit"):
		clientsock.close()
	#clientsock.close()

def console_input(clientsock,addr):
   while True:
      msg = raw_input()
      print 'sending message to android'
      clientsock.send(msg + '\n')

def say(soundfile):
			if soundfile == "":
				return
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

"""
every time there is an update on 'setRobot' publishes a message about new state
of the robots in /context_features
"""
def updateContext(state):
	global contextPub

	message = RobotFeaturesMsg()
	message.robotTalking = 0
	message.robotBouncing = 0

	if state == 'speak':
		message.robotTalking = 1
	if state == 'bounce':
		message.robotBouncing = 1

	contextPub.publish(message)

	
def setRobot(launchID, state, dur):
	message = ControllerBehaviorMsg()
	message.ID = launchID
	message.action = state
	message.duration = dur
	pub.publish(message)

	updateContext(state)

	logAction("animation", state, launchID)

def lookAt(robot, direction):
	print "ROBOT " + str(robot) + " looking at " + direction
	if direction == 'left':
		if robot == 1:
			setRobot(1, "pan", random.randint(50,60))
		if robot == 2:
			setRobot(2, "pan", random.randint(70,80))
	if direction == 'middle':
		if robot == 1:
			setRobot(1, "pan", random.randint(90,100))
		if robot == 2:
			setRobot(2, "pan", random.randint(100,110))
	if direction == 'right':
		if robot == 1:
			setRobot(1, "pan", random.randint(110,120))
		if robot == 2:
			setRobot(2, "pan", random.randint(130,140))

def timeoutPassed(oldepoch):
    return time.time() - oldepoch >= TIMEOUT_SAY_AGAIN

def getScenebyID(sceneList, sceneID):
	for key in sceneList:
		if key["SceneID"] == sceneID:
			return key

def getScenebyName(sceneList, sceneName):
	for key in sceneList:
		if key["SceneName"] == sceneName:
			return key

def sortEngagement(): #sort from smallest disengagement to largest
	global engagelist

	engagelist = dict()

	#create a dictionary of 'left','right','middle' with disengagement values as key
	#handles duplicate disengagement values by putting the positions in lists
	engagelist[dis_left] = ['left'];

	if dis_right in engagelist:
		engagelist[dis_right].append('right')
	else:
		engagelist[dis_right] = ['right']

	if dis_middle in engagelist:
		engagelist[dis_middle].append('middle')
	else:
		engagelist[dis_middle] = ['middle']

	# engagelist = []
	# engagelist.append(('left',dis_left))
	# if dis_middle < dis_left:
	# 	engagelist.insert(0, ('middle',dis_middle))
	# 	if dis_right < dis_middle:
	# 		engagelist.insert(0, ('right',dis_right))
	# 	elif dis_right < dis_left:
	# 		engagelist.insert(1, ('right',dis_right))
	# 	else:
	# 		engagelist.insert(2, ('right',dis_right))
	# else:
	# 	engagelist.insert(1, ('middle',dis_middle))
	# 	if dis_right < dis_left:
	# 		engagelist.insert(0, ('right',dis_right))
	# 	elif dis_right < dis_middle:
	# 		engagelist.insert(1, ('right',dis_right))
	# 	else:
	# 		engagelist.insert(2, ('right', dis_right))



#improve!
def playInterrupt():
	global dis_left, dis_middle, dis_right
	global intScript
	global engagelist
	###TEST
	global CHOICE

	nrStrategies = int(intScript["NrScenes"])
	choice = random.randint(0,nrStrategies-1)
	strategyList = intScript["Strategies"]
	strategy = strategyList[choice]
	###TEST
	#strategy = strategyList[CHOICE % nrStrategies]
	print strategy["StrategyName"]
	actionList = strategy["SceneActions"]

	key = 0
	while key < len(actionList):
		if actionList[key]["expression"] == "pan":
			setRobot(actionList[key]["actor"], actionList[key]["expression"], actionList[key]["sound"])
			logAction("pan", str(actionList[key]["sound"]), actionList[key]["actor"])
			print 'pan ' + str(actionList[key]["sound"]) + ' degrees'
		elif actionList[key]["expression"] == "lookat":
			print engagelist
			if actionList[key]["sound"] == "engaged": #choose child at random from lowest disengagement values
				print engagelist[min(engagelist)]
				print 'seed' + str(last_interrupt_time)
				random.seed(last_interrupt_time)  #make sure both robots look at same person
				lookAt(actionList[key]["actor"],random.choice(engagelist[min(engagelist)]))
			else:	#choose child at random from highest disengagement values
				print 'seed' + str(last_interrupt_time)
				random.seed(last_interrupt_time)
				lookAt(actionList[key]["actor"],random.choice(engagelist[max(engagelist)]))
			sleep(0.5)
		else:
			setRobot(actionList[key]["actor"], actionList[key]["expression"],0)
			logAction("sound", str(actionList[key]["sound"]), actionList[key]["actor"])
			say(actionList[key]["sound"])

		if actionList[key]["expression"] == "bounce" or actionList[key]["expression"] == "pan":
			#bounce for N seconds before getting back to idle, or allow time to pan 
			sleep(PAUSE + actionList[key]["pause"]) 
		else:  #go back to idle and pause for a while
			setRobot(actionList[key]["actor"], "idle", 0)
			sleep(PAUSE + actionList[key]["pause"])
		key+=1

	###TEST
	CHOICE+=1
	print 'done playing interrupt'

def checkDisengagement():
	global dis_left, dis_middle, dis_right, interactionState
	global engagelist
	global nr_interruptions, last_interrupt_time
	global last_message_time
	global MAX_INTERRUPTIONS, MIN_TIME

	current_time = timer()

	#decay stuff
	time_decay = int(current_time - last_message_time)
	dis_left -= DECAY_FACTOR*time_decay
	dis_right -= DECAY_FACTOR*time_decay
	dis_middle -= DECAY_FACTOR*time_decay

	print 'decay time: ' + str(time_decay)

	if dis_middle < 0:
		dis_middle = 0
	if dis_left < 0:
		dis_left = 0
	if dis_right < 0:
		dis_right = 0

	if time_decay > 0:
		print 'Checking disengagement: child left: ' + str(dis_left) + '\tchild middle: ' + str(dis_middle) \
		+ '\t child right: ' + str(dis_right)
	sortEngagement()
	


	#CHECK still resets values after ignoring the interrupt
	if dis_left > DIS_THRESHOLD or dis_middle > DIS_THRESHOLD or dis_right > DIS_THRESHOLD:
		dis_left = 0
		dis_middle = 0
		dis_right = 0
		if nr_interruptions < MAX_INTERRUPTIONS and (current_time - last_interrupt_time) > MIN_TIME:
			interactionState = STATE_INTERRUP
			nr_interruptions += 1
			last_interrupt_time = current_time
			print 'trigering disengagement'
		else:
			print 'ignored disengagement'
		
def playSceneActions(actionList):
	global interruptState, interactionState
	global nr_interruptions, dis_right, dis_left, dis_middle

	#check if these two are still needed
	setRobot(1, "pan", 90)
	setRobot(2, "pan", 90)

	sleep(0.3)
	key = 0
	restart = 0

	nr_interruptions = 0

	#do not let values get too negative, reset all to 0 at the beginning of each scene
	dis_left = 0
	dis_middle = 0
	dis_right = 0

	while key < len(actionList):
		if actionList[key]["restart"] == 1:   #set most recent place to restart
			restart = key

		if interruptMode != 0:
			checkDisengagement()

		if interactionState == STATE_INTERRUP:
			print 'INTERRUPTED at ', key
			#interruptState = 0
			#sleep(3)
			playInterrupt()
			interactionState = STATE_PLAYING
			key = restart

		if not actionList[key]["expression"] == "pan": 
			setRobot(actionList[key]["actor"], actionList[key]["expression"],0)
			logAction("sound", str(actionList[key]["sound"]), actionList[key]["actor"])
			say(actionList[key]["sound"])
		else:  # new code to allow pan
			setRobot(actionList[key]["actor"], actionList[key]["expression"], actionList[key]["sound"])
			logAction("pan", str(actionList[key]["sound"]), actionList[key]["actor"])
			print 'pan ' + str(actionList[key]["sound"]) + ' degrees'

		if actionList[key]["expression"] == "bounce" or actionList[key]["expression"] == "pan":
			#bounce for N seconds before getting back to idle, or allow time to pan 
			sleep(PAUSE + actionList[key]["pause"]) 
		else:  #go back to idle and pause for a while
			setRobot(actionList[key]["actor"], "idle", 0)
			sleep(PAUSE + actionList[key]["pause"])
		key+=1

def interactionScript(clientsock, b):
	global interactionState
	global currentSceneName
	global last_message_time

	last_message_time = timer()
	while interactionState == STATE_WAITING_START:
		x = 0
	
	global script

	storyName = script["StoryName"]
	nrScenes = int(script["NrScenes"])
	sceneList = script["Scenes"]

	if nrScenes > 0:
		currentSceneName = "Intro"
		while 1:
			if (interactionState == STATE_PLAYING):
				currentScene = getScenebyName(sceneList, currentSceneName)
				playSceneActions(currentScene["SceneActions"])
				nextScenes = currentScene["NextScenes"]
				interactionState = STATE_WAITING_CHOICE
				logAction("state", "STATE_WAITING_CHOICE", 0)
				clientsock.send('showOptions\n')
				print "sending command to tablet"
			if (interactionState == STATE_CHOICE_MADE):
				interactionState = STATE_PLAYING
				logAction("state", "STATE_PLAYING", 0)

#publishes log messages in a ros topic to be recorded by a rosbag
#to do: record also in a text file to have a cleaner version just in case
def logAction(type, desc, robot):
	global logPub

	#print "(Action) Now since beggining: " + str(rospy.get_rostime().secs - interactionStartTime)
	message = LogMsg()
	if interactionStartTime == 0:
		message.time = 0  #change later
	else:
		message.time = rospy.get_rostime().secs - interactionStartTime
	message.type = type
	message.desc = desc
	message.robotID = robot
	logPub.publish(message)

#publishes log messages in a ros topic to be recorded by a rosbag
#to do: record also in a text file to have a cleaner version just in case
def logDisengagement(feats_child_left, feats_child_middle, feats_child_right, pred_child_left, pred_child_middle, pred_child_right):
	global logDisPub, interactionStartTime

	#print "(Diseng) Now since beggining: " + str(rospy.get_rostime().secs - interactionStartTime)
	message = LogDisengagementMsg()
	if interactionStartTime == 0:
		message.time = 0  #change later
	else:
		message.time = rospy.get_rostime().secs - interactionStartTime

	message.pred_child_left = pred_child_left
	message.pred_child_middle = pred_child_middle
	message.pred_child_right = pred_child_right
	message.feats_child_left = feats_child_left
	message.feats_child_middle = feats_child_middle
	message.feats_child_right = feats_child_right

	logDisPub.publish(message)

def loadScript(storyName):
	global scriptfile, script

	fileLocation = 'script-' + storyName + '/script.json'
	try:
		with open(fileLocation):
			scriptfile = open(fileLocation, 'r')
			script = json.load(scriptfile)
			print 'Opening exiting script file: ' + str(scriptfile)
	except IOError:
		print 'Unable to open script file!'

def loadInterruptions(interruptType):
	global intScriptfile, intScript

	fileLocation = 'dis-strategies/dis-'+ interruptType + '.json'
	try:
		with open(fileLocation):
			intscriptfile = open(fileLocation, 'r')
			intScript = json.load(intscriptfile)
			print 'Opening interruption script file: ' + str(intscriptfile)
	except IOError:
		print 'Unable to open interruption script file!'

def interruption(data): #HERE
	global interruptState
	interruptState = int(data.data)


'''
receives diseng values for each child
updates dis variables and, if necessary, changes STATE_INTERRUP variable
'''
def DisPredictionCallback(msg):
	global dis_left, dis_middle, dis_right

	global last_message_time

	last_message_time = timer()

	print 'MESSAGE: child left: ' + str(msg.pred_child_left) + '\tchild middle: ' + str(msg.pred_child_middle) \
	+ '\tchild right: ' + str(msg.pred_child_right)
	print '-'

	if (interactionState == STATE_PLAYING):
		if msg.pred_child_left == 1:
			dis_left+=1
		elif msg.pred_child_left == 0:
			dis_left-=2

		if msg.pred_child_middle == 1:
			dis_middle+=1
		elif msg.pred_child_middle == 0:
			dis_middle-=2

		if msg.pred_child_right == 1:
			dis_right+=1
		elif msg.pred_child_right == 0:
			dis_right-=2
	else:
		print 'Not playing'

	print 'child left: ' + str(dis_left) + '\tchild middle: ' + str(dis_middle) \
	+ '\t child right: ' + str(dis_right)
	print '----'

	logDisengagement(msg.feats_child_left, msg.feats_child_middle, msg.feats_child_right, \
	msg.pred_child_left, msg.pred_child_middle, msg.pred_child_right)

#manage the decays for dis variables and 'fire interruptions'
def interruptManager(arg1, arg2):
	global dis_left, dis_middle, dis_right, interruptState

	while (1):
		#print 'in thread interrupt'
		if dis_left > DIS_THRESHOLD or dis_middle > DIS_THRESHOLD or dis_right > DIS_THRESHOLD:
			interruptState = STATE_INTERRUP

def main():
	global pub, logPub, contextPub, disengagementSubscriber, logDisPub
	global i
	global s #socket
	
	global interactionState
	global scriptfile, script
	global interactionStartTime
	
	global interruptState  #HERE
	global intScriptfile, intScript
	global interruptMode

	interruptMode = 0
	interruptState = 0

	
	interactionState = STATE_WAITING_START

	print '**************************************'
	print '******    STORY CONTROLLER  **********'
	print '**************************************'
	
	# Initialize ROS node
	rospy.init_node('controller')
	
	# Create publisher to send out ControllerMsg messages
	pub = rospy.Publisher('EIcontroller_data', ControllerBehaviorMsg)
	rospy.loginfo("EI Controller node inititalized.")
	rospy.sleep(2.0) #important!

	# Create publisher to send out ControllerMsg messages
	contextPub = rospy.Publisher('EI_context_features', RobotFeaturesMsg, queue_size=2)
	rospy.loginfo("EIcontroller_context node inititalized.")

	# Create publisher to send out ControllerMsg messages
	logPub = rospy.Publisher('ei_log_data', LogMsg, queue_size=2)
	rospy.loginfo("Log publisher initialized.")

	# Create publisher to send out ControllerMsg messages
	logDisPub = rospy.Publisher('dis_log_data', LogDisengagementMsg, queue_size=2)
	rospy.loginfo("Disengagement Log publisher initialized.")

	disengagementSubscriber = rospy.Subscriber('/disengagement_prediction', DisengagementPrediction, DisPredictionCallback)

	'''
	change to Subscribe to disengagement
	'''
	rospy.Subscriber("interrupt", String, interruption) #HERE
	
	rospy.sleep(1.0) #important!


	interactionStartTime = 0
	#nsecs_interactionStartTime = 0

	#Initialize socket to connect with haldheld device
	signal.signal(signal.SIGINT, signal_handler)
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

	#new line to avoid waiting for binding
	s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

	#s.settimeout(None)

	print 'Socket created' 
	#Bind socket to local host and port
	try:
		s.bind((HOST, PORT))
	except socket.error as msg:
		print 'Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
		sys.exit()
    
	print 'Socket bind complete'

	#s.settimeout(100000000000)

	s.listen(10)
	print 'Socket now listening'

	
	rospy.sleep(2.0) #important!
 
	#***************************************for test only!

	if len(sys.argv) > 1:
		if sys.argv[1] == 'intro':
			print 'RUN INTRO SCRIPT *************'
			loadScript('intro')
			interactionState = STATE_PLAYING
			thread.start_new_thread(interactionScript,(1,1))
		elif sys.argv[1] == 'individual':
			loadInterruptions('individual')
			interruptMode = 1
		elif sys.argv[1] == 'group':
			loadInterruptions('group')
			interruptMode = 2

	#thread.start_new_thread(interactionScript,(2,1))
 
	#now keep talking with the client
	while 1:
		try:
			#wait to accept a connection - blocking call
			print 'waiting for tablet connection\n'
			conn, addr = s.accept()
			print 'Connected with ' + addr[0] + ':' + str(addr[1])
			interactionState = STATE_WAITING_START
			thread.start_new_thread(interactionScript,(conn,1))
			thread.start_new_thread(socketHandler, (conn, addr))
		#thread.start_new_thread(interruptManager, ('',''))
		except socket.timeout:
			print 'timeout exception on main'

	while not rospy.is_shutdown():
		rospy.sleep(2.0)
		interactionScript(pub,2)

		
if __name__ == '__main__':
	main()
