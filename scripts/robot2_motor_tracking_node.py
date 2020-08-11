#!/usr/bin/env python

# import the necessary packages
#from collections import deque
#from tensorflow.keras.preprocessing.image import img_to_array
#from tensorflow.keras.models import load_model

#import tensorflow as tf
#import imutils
#import time
#import cv2
#import os
#import rospkg
#import sys
import rospy
import numpy as np

# import the necessary ROS messages 
#from std_msgs.msg import String
#from sensor_msgs.msg import Image

from geometry_msgs.msg import Twist
from sensor_msgs.msg import CameraInfo
from common_face_application.msg import objCenter as objCoord

#from cv_bridge import CvBridge
#from cv_bridge import CvBridgeError



class RobotVisionNavi:

	def __init__(self):

		rospy.logwarn("[Robot2] Vision Navigation (ROI) node [ONLINE]")
		
		self.move = Twist()
		self.getInfo = False
		self.getCoord = False

		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)
		
		# Subscribe to CameraInfo msg
		cameraInfo_topic = "/cv_camera_robot2/camera_info"
		self.cameraInfo_sub = rospy.Subscriber(cameraInfo_topic, CameraInfo, 
			self.cbCameraInfo)
		
		# Publish to twist msg
		twist_topic = "/cmd_vel_robot2"
		self.twist_pub = rospy.Publisher(twist_topic, Twist, queue_size=10)
		
		# Subscribe to objCenter msg
		objCoord_topic = "/objCoord_robot2"
		self.objCoord_sub = rospy.Subscriber(objCoord_topic, objCoord, 
			self.cbObjcoord)
			
#		# Allow up to one second to connection
		rospy.sleep(1)
		
	# Get CameraInfo
	def cbCameraInfo(self, msg):
		try:
			self.imgWidth = msg.width
			self.imgHeight = msg.height

		except KeyboardInterrupt as e:
			print(e)

		if self.imgWidth is not None and self.imgHeight is not None:
			self.getInfo = True
		else:
			self.getInfo = False

		# calculate the center of the frame as this is where we will
		# try to keep the object

		self.centerX = self.imgWidth // 2
		self.centerY = 3 * (self.imgHeight // 4)
	
	def cbObjcoord(self, msg):
		try:
			self.objX = msg.centerX
		 	self.objY = msg.centerY
	 	except KeyboardInterrupt as e:
	 		print(e)
	 		
	 	if self.objX is not None and self.objY is not None:
	 		self.getCoord = True
 		else:
			self.getCoord = False
	
	def cbNavigation(self):
		if self.getInfo and self.getCoord:
#			rospy.loginfo([self.imgWidth, self.imgHeight])
#			rospy.loginfo([self.objX, self.objY])

			errDC = self.centerY - self.objY
		
			if (errDC>0) :
				self.move.linear.x = 0.05
				self.move.linear.y = 0.00
				self.move.linear.z = 0.00
			
				self.move.angular.x = 0.00
				self.move.angular.y = 0.00
				self.move.angular.z = 0.00
			
				self.twist_pub.publish(self.move)
			
			
			else :
				self.move.linear.x = 0.00
				self.move.linear.y = 0.00
				self.move.linear.z = 0.00
			
				self.move.angular.x = 0.00
				self.move.angular.y = 0.00
				self.move.angular.z = 0.00
			
				self.twist_pub.publish(self.move)

		else:
#			rospy.logerr("No Info Received!")
			pass
			
		# Allow up to one second to connection
		rospy.sleep(0.1)			
			
	# rospy shutdown callback
	def cbShutdown(self):
		try:
			rospy.logwarn("[Robot2] Vision Navigation (ROI) node [OFFLINE]")
		finally:
			self.move.linear.x = 0
			self.move.linear.y = 0
			self.move.linear.z = 0
			
			self.move.angular.x = 0
			self.move.angular.y = 0
			self.move.angular.z = 0
			
			self.twist_pub.publish(self.move)
			
			
if __name__ == '__main__':

	# Initializing your ROS Node
	rospy.init_node('robot2_navigation_camera_based', anonymous=False)
	nav = RobotVisionNavi()

	# Camera preview
	while not rospy.is_shutdown():
		nav.cbNavigation()

