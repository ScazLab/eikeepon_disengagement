#!/usr/bin/env python


"""
This node listens for incoming messages from Windows kinect topics (published via ROSbridge)
parses the incoming data to identify what kind of data
"""

import roslib; roslib.load_manifest('eikeepon_pkg')

import rospy 
from eikeepon_pkg.msg import DisFeaturesMsg, DisengagementPrediction, RobotFeaturesMsg

#import argparse
#from read_data import *

#sklearn imports 
from sklearn import svm
from sklearn import metrics

#for saving models
import pickle
from sklearn.externals import joblib

LEFT = 0
MIDDLE = 1
RIGHT = 2

"""
indexes of message from kinect parser
"""
IND_ENGAGED = 0
IND_TALKING = 1
IND_SMILING = 2
IND_FORWARD = 3
IND_BACKWARD = 4
IND_LOOKUP = 5
IND_LOOKDOWN = 6
IND_LOOKROBOTS = 7

dis_model_path = '/home/keepon/eikeepon/branches/nathaya-branch/trunk/src/eikeepon_pkg/src/models/model_linux_19_05_15.pkl'

class DisengagemenClassifier:
	#keeps updating through /context_updates_callback
	robots_talking = 0
	robots_bouncing = 0

	def __init__(self):
		self.node_name = 'disengagement_classifier'

		#load disengagement model classifier
		self.clf = joblib.load(dis_model_path)

		rospy.init_node(self.node_name)

		#Subscribe to the disengagement_feature messages from Kinect Parser
		self.sub_kinect = rospy.Subscriber('/disengagement_features', DisFeaturesMsg, self.disengagement_classification_callback)
		
		#Subscribe to the disengagement_feature messages from Kinect Parser
		self.sub_context = rospy.Subscriber('/EI_context_features', RobotFeaturesMsg, self.context_updates_callback)

		# Publishers
		##### Disengagement prediction for each children in the group
		self.pub_disengagement = rospy.Publisher('/disengagement_prediction', DisengagementPrediction, queue_size=2)


	def run(self):
		rospy.spin()

	"""
	def predict(self, X):
		y_pred = self.clf.predict(X)
		return y_pred
	"""


	"""
	predicts the disengagement value for each child and publishes the result in /disengagement_prediction
	"""
	def disengagement_classification_callback (self, msg):
		print 'receiving a message from /disengagement_features'
		#print 'child left: ' + msg.child_left
		#print 'child right: ' + msg.child_middle
		#print 'child right: ' + msg.child_right
		#print 'robot talking? ' + str(self.robots_talking)
		#print 'robot bouncing?' + str(self.robots_bouncing)

		x_left = self.append_features_one_line(msg.feats_child_left)
		x_middle = self.append_features_one_line(msg.feats_child_middle)
		x_right = self.append_features_one_line(msg.feats_child_right)

		print 'left ' + str(x_left)
		print 'middle ' + str(x_middle)
		print 'right ' + str(x_right)
		print '\n'

		predictions = DisengagementPrediction()
		predictions.pred_child_left = self.clf.predict(x_left)
		predictions.pred_child_middle = self.clf.predict(x_middle)
		predictions.pred_child_right = self.clf.predict(x_right)
		predictions.feats_child_left = msg.feats_child_left
		predictions.feats_child_middle = msg.feats_child_middle
		predictions.feats_child_right = msg.feats_child_right

		self.pub_disengagement.publish(predictions)
		#to do
		#predict and publish results for each child

	def context_updates_callback (self, msg):
		#print 'receiving a message from /context_features'	
		#print 'talking: ' + str(msg.robotTalking)
		#print 'bouncing: ' + str(msg.robotBouncing)
		self.robots_talking = msg.robotTalking
		self.robots_bouncing = msg.robotBouncing

	"""
	appends kinect + context features and returns a x[]
	"""
	def append_features_one_line (self, kinect_feats):
		x = []

		x.append(int(kinect_feats[IND_TALKING]))
		x.append(int(kinect_feats[IND_SMILING]))
		x.append(int(kinect_feats[IND_FORWARD]))
		x.append(int(kinect_feats[IND_BACKWARD]))
		x.append(self.robots_talking)
		x.append(self.robots_bouncing)
		x.append(int(kinect_feats[IND_LOOKUP]))
		x.append(int(kinect_feats[IND_LOOKDOWN]))
		x.append(int(kinect_feats[IND_LOOKROBOTS]))

		return x

	#returns a training set with all data except 'leaveoOut' participant
	def append_features_except (data, leaveOut):
		X = []
		y = []
		for participant in data.keys():
			if participant == leaveOut:
				#print 'excluding participant ' + participant
				continue
			#print participant
			for i in data[participant]:
				y.append(i[0])
				X.append(i[1])
		return X,y

	#returns all y and X values for one participant
	def append_features_participant (data, participant_key):
		X = []
		y = []
		for participant in data.keys():
			if participant == participant_key:
				for i in data[participant]:
					y.append(i[0])
					X.append(i[1])
		return X,y

	def leave_one_out_cross_validation(data):
		mean_accuracy_list = []
		f1_score_list = []
		roc_auc_list = []

		#y_true = []
		y_score_prob = []
		y_pred = []

		all_y_pred = []
		all_y_true = []
		all_y_pred_proba = []

		pred_results = open('prediction_results.txt', "w")

		for participant in data.keys():
		
			print 'for participant ' + participant
		
			X_train,y_train = append_features_except(data, participant)
			X_test, y_test = append_features_participant(data, participant)
					
			# Run classifier
			model = svm.SVC(kernel='rbf', C=4.0, gamma=0.5, class_weight='auto', probability=True)
			y_pred = model.fit(X_train, y_train).predict(X_test)

			y_pred_proba = model.fit(X_train, y_train).predict_proba(X_test)

			all_y_pred.append(y_pred)
			all_y_true.append(y_test)
			all_y_pred_proba.append(y_pred_proba)

			# metrics
			mean_accuracy = model.score(X_test, y_test)
			mean_accuracy_list.append(mean_accuracy)

			#fpr,tpr,thresholds = metrics.roc_curve(y_test, y_pred, pos_label=-1)
			fpr,tpr,thresholds = metrics.roc_curve(y_test, y_pred, pos_label=1)
		
			roc_auc = metrics.auc(fpr, tpr)
			roc_auc_list.append(roc_auc)

			f1_score = metrics.f1_score(y_test, y_pred)
			f1_score_list.append(f1_score)

			print "\tMean Accuracy:\t\t\t" + str(mean_accuracy)
			print "\tArea under the ROC curve:\t %f" % roc_auc
			print "\tF1 Score:\t\t\t" + str(f1_score) + '\n'

			for i in range(len(y_test)):
				pred_results.write(participant + '\t' + str(y_test[i]) + '\t' + str(y_pred[i]) + '\t' + str(y_pred_proba[i]) + '\n') 
       
		average_mean_accuracy = sum(mean_accuracy_list)/len(mean_accuracy_list)
		average_f1_score = sum(f1_score_list)/len(f1_score_list)
		average_roc_auc = sum(roc_auc_list)/len(roc_auc_list)

		print "Mean Accuracy:\t\t\t" + str(average_mean_accuracy)
		print "Mean F1 Score:\t\t\t" + str(average_f1_score)
		print "Mean AUC Score:\t\t\t" + str(average_roc_auc)

		pred_results.close()
		return [average_mean_accuracy, average_f1_score, average_roc_auc]

	#returns X and y to build a large dataset with all data
	def append_all_features(data):
		X = []
		y = []
		for participant in data.keys():
			for i in data[participant]:
				y.append(i[0])
				X.append(i[1])
		print 'lentgh of dataset: X: ' + str(len(X)) + '     Y: ' + str(len(X)) + '\n'
		return X,y
	#def dis_build_model

	def build_model(model_name, data):
		X_train,y_train = append_all_features(data)

		clf = svm.SVC(kernel='rbf', C=4.0, gamma=0.5, class_weight='auto', probability=True)
		clf.fit(X_train, y_train)
		#model = pickle.dumps(clf)
		joblib.dump(clf, model_name)

if __name__ == '__main__':
	disClass = DisengagemenClassifier()
	disClass.run()

""" 
#def dis_predict
if __name__ == '__main__':
	parser = argparse.ArgumentParser(description='disengagement classification module')
	parser.add_argument('mode', default='CV', #CV or BM
                        help='CV for Cross Validation, BM for Building a Model')
	parser.add_argument('feature_file', default='data/output_data_SD.csv', 
                        help='filename of X data')
	parser.add_argument('model_name', help='model filename', nargs='?')
	args = parser.parse_args()

	if args.model_name:
		model_name = args.model_name + '.pkl'  # save model name

	print '\nLoading data from file: ' + args.feature_file + '\n'
	data = load_data_libsvm_format(args.feature_file)
	
	if args.mode == 'CV':
		print '\nPerforming Cross Validation ... '
		leave_one_out_cross_validation(data)
	elif args.mode == 'BM':
		print 'building model...'
		build_model(model_name, data)
	else:
		print 'NO VALID MODE! \n Use CV for Cross Validation, BM for Building a Model'
"""

	#testing the built model:
	# clf = joblib.load(model_name)
	# testfile = open('persistentModel_prediction_results.txt', "w")

	# for participant in data.keys():
	# 	testfile.write(participant + '\n')
	# 	for line in data[participant]:
	# 		#print str(i[0]) + ' ' + str(i[1])
	# 		y_pred = predict(clf, line[1])
	# 		testfile.write('label: '+ str(line[0]) + '\t y_pred: ' + str(y_pred[0]) + '\n')


	#for participant in data.keys():
	#	print participant
	#	for i in data[participant]:
	#		print str(i[0]) + ' ' + str(i[1])