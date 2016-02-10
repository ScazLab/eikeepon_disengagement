#!/usr/bin/env python


"""
This node listens for incoming messages from Windows kinect topics (published via ROSbridge)
parses the incoming data to identify what kind of data
"""

import roslib; roslib.load_manifest('eikeepon_pkg')
import rospy

from pprint import pprint
from std_msgs.msg import String
from eikeepon_pkg.msg import DisFeaturesMsg, RobotFeaturesMsg

import thread
import socket
import sys
import signal

DEBUG = 1

HOST = ''   # Symbolic name, meaning all available interfaces
PORT = 1200 # Arbitrary non-privileged port
BUFSIZE = 1024

LEFT = 0
MIDDLE = 1
RIGHT = 2

def debug(msg):
    if DEBUG:
        print msg

"""
Publishes on: /disengagement_features
"""

class KinectParser():

    clientsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 

    def __init__(self):
        self.node_name = 'kinect_parser'
        rospy.init_node(self.node_name)
        
        # Publishers
        ##### Add new publishers for topics here #####
        self.pub_features = rospy.Publisher('/disengagement_features', DisFeaturesMsg, queue_size=2)

        #rospy.sleep(5.0) #important!


    def socketHandler(self, conn):

      while 1:
        try:
          data = conn.recv(BUFSIZE)

          if not data:
            break
          
          print 'data from socket ' + data

          oneStream = data.split('$')
          #print 'one msg only:  ' + oneStream[0] + '\n'

          features = oneStream[0].split(':')
          message = DisFeaturesMsg()
          message.feats_child_left = features[LEFT]
          message.feats_child_middle = features[MIDDLE]
          message.feats_child_right = features[RIGHT]

          #print 'left: ' + message.child_left
          #print 'middle: ' + message.child_middle
          #print 'right: ' + message.child_right + '\n'

          self.pub_features.publish(message)

        except socket.error as msg:
          print 'Error receiving data: ' + str(msg[0]) + '. Message ' + msg[1]
         # if not data:
          #  break

    def signal_handler(self, signal, frame):
      print 'Closing the socket and exiting...'
      try:
       self.clientsock.close()
      except socket.error as msg:
        print 'Closing scoket failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
        sys.exit()
      sys.exit(0)

    def run(self):
      print ' on run'

      self.clientsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

      print 'Socket created' 

      #Bind socket to local host and port
      try:
        self.clientsock.bind((HOST, PORT))
      except socket.error as msg:
        print 'Bind failed. Error Code : ' + str(msg[0]) + '. Message ' + msg[1]
        sys.exit()

      print 'Socket bind complete'
      self.clientsock.listen(10)
      print 'Socket now listening'

        #Initialize socket to connect with haldheld devic
      signal.signal(signal.SIGINT, self.signal_handler)


      print '**************************************'
      print '******    KINECT PARSER     **********'
      print '**************************************'

      while 1:
        print 'waiting for Windows connection\n'
        conn, addr = self.clientsock.accept()
        print 'Connected with ' + addr[0] + ':' + str(addr[1])
        self.socketHandler(conn)

      rospy.spin()

if __name__ == '__main__':
    parseNode = KinectParser()
    parseNode.run()
