#!/usr/bin/env python

# import the necessary packages
import rospy
import numpy as np

# import the necessary ROS messages 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CameraInfo
from common_face_application.msg import objCenter as objCoord

class RobotVisionNavi:

	def __init__(self):

		rospy.logwarn("Robot Vision (ROI) node [ONLINE]")
		
		self.move = Twist()

		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)
		
		# Subscribe to objCenter msg
		objCoord_topic = "/objCoord_robot1"
		self.objCoord_sub = rospy.Subscriber(objCoord_topic, objCoord, self.cbObjcoord)
		
		# Subscribe to CameraInfo msg
		cameraInfo_topic = "/cv_camera/camera_info"
		self.cameraInfo_sub = rospy.Subscriber(cameraInfo_topic, CameraInfo, self.cbCameraInfo)
		
		# Publish to twist msg
		twist_topic = "/cmd_vel_robot1"
		self.twist_pub = rospy.Publisher(twist_topic, Twist, queue_size=10)


#		# Allow up to one second to connection
		rospy.sleep(0.1)
		
	# Get CameraInfo
	def cbCameraInfo(self, msg):

		self.imgWidth = msg.width
		self.imgHeight = msg.height

		# calculate the center of the frame as this is where we will
		# try to keep the object

		self.centerX = self.imgWidth // 2
		self.centerY = self.imgHeight // 2
	
	def cbObjcoord(self, msg):
		
		self.objX = msg.centerX
	 	self.objY = msg.centerY
		
	
	def cbNavigation(self):
#		rospy.loginfo([self.imgWidth, self.imgHeight])
#		rospy.loginfo([self.objX, self.objY])
#		
		errDC = self.centerY - self.objY
		
		if (errDC>0) :
			self.move.linear.x = 0.5
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
			
			
	# rospy shutdown callback
	def cbShutdown(self):
		try:
			rospy.logwarn("MOVE (ROI) node [OFFLINE]")
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
	rospy.init_node('robot1_navigation_camera_based', anonymous=False)
	nav = RobotVisionNavi()

	# Camera preview
	while not rospy.is_shutdown():
		nav.cbNavigation()

