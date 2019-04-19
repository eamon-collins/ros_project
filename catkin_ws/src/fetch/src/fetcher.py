#!/usr/bin/env python
import roslib
#roslib.load_manifest('turtle_tf')
import rospy

import math
import tf
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
import turtlesim.srv
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt, tan
import os
from cv_bridge import CvBridge, CvBridgeError
import cv2




class State():
	def __init__(self, state, odom):
		self.myState = state
		self.myOdom = odom
		self.bridge = CvBridge()


myState = State("findBalls", Odometry())

def update_odom(msg):
	myState.myOdom = msg

def handle_scan(msg):
	pass

def handle_image(msg):
	try:
		cv_image = myState.bridge.imgmsg_to_cv2(msg, "bgr8")
	except CvBridgeError as e:
		print(e)
	
	(rows, cols, channels) = cv_image.shape
	if cols > 60 and rows > 60:
		cv2.circle(cv_image, (50,50),10,255)

	cv2.imshow("view from turtlebot3", cv_image)
	cv2.waitKey(3)


if __name__ == '__main__':
	rospy.init_node('fetcher')

	


	turtle_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	pose_read = rospy.Subscriber('/odom', Odometry, update_odom)
	scan = rospy.Subscriber('/scan', LaserScan, handle_scan)
	camera = rospy.Subscriber('/camera/rgb/image_raw', Image, handle_image)
	


	rate = rospy.Rate(20)
	while not rospy.is_shutdown():

		

		rate.sleep()
		#rospy.spin()