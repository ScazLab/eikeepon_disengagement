#!/usr/bin/env python

import rospy, signal, sys
from std_msgs.msg import String

from eikeepon_pkg.msg import DisengagementPrediction

	
def signal_handler(signal, frame):
	print 'Closing the debug module...'
	sys.exit(0)


def talker():
	#pub = rospy.Publisher('interrupt', String, queue_size=10)
	pub = rospy.Publisher('/disengagement_prediction', DisengagementPrediction, queue_size=2)

	rospy.init_node('talker', anonymous=True)

	message_dis_all = DisengagementPrediction()
	message_dis_all.pred_child_left = 1
	message_dis_all.pred_child_middle = 1
	message_dis_all.pred_child_right = 1
	message_dis_all.feats_child_right = ""
	message_dis_all.feats_child_middle = ""
	message_dis_all.feats_child_left = ""

	message_eng_all = DisengagementPrediction()
	message_eng_all.pred_child_left = 0
	message_eng_all.pred_child_middle = 0
	message_eng_all.pred_child_right = 0
	message_eng_all.feats_child_right = ""
	message_eng_all.feats_child_middle = ""
	message_eng_all.feats_child_left = ""

	message_dis_left = DisengagementPrediction()
	message_dis_left.pred_child_left = 1
	message_dis_left.pred_child_middle = -1
	message_dis_left.pred_child_right = -1
	message_dis_left.feats_child_right = ''
	message_dis_left.feats_child_middle = ''
	message_dis_left.feats_child_left = ''

	message_dis_right = DisengagementPrediction()
	message_dis_right.pred_child_left = -1
	message_dis_right.pred_child_middle = -1
	message_dis_right.pred_child_right = 1
	message_dis_right.feats_child_right = ''
	message_dis_right.feats_child_middle = ''
	message_dis_right.feats_child_left = ''

	message_dis_middle = DisengagementPrediction()
	message_dis_middle.pred_child_left = -1
	message_dis_middle.pred_child_middle = 1
	message_dis_middle.pred_child_right = -1
	message_dis_middle.feats_child_right = ''
	message_dis_middle.feats_child_middle = ''
	message_dis_middle.feats_child_left = ''

	while not rospy.is_shutdown():
		i= raw_input()

		if i == 'd':
			pub.publish(message_dis_all)
			print "disengagement WoZ: sending all disengaged"
		if i == 'l':
			pub.publish(message_dis_left)
			print "disengagement WoZ: sending disengaged left child"
		if i == 'm':
			pub.publish(message_dis_middle)
			print "disengagement WoZ: sending disengaged middle child"
		if i == 'r':
			pub.publish(message_dis_right)
			print "disengagement WoZ: sending disengaged right child"
		if i == 'e':
			pub.publish(message_eng_all)
			print "disengagement WoZ: sending ENGAGED message"


if __name__ == '__main__':
	signal.signal(signal.SIGINT, signal_handler)
	
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
