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
import numpy as np




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

	gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
	balls = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1, minDist=100, param1=100, param2=30, minRadius=5, maxRadius=250)
	print(balls)

	if balls is not None:
		# convert the (x, y) coordinates and radius of the circles to integers
		balls = np.round(balls[0, :]).astype("int")
	 
		# loop over the (x, y) coordinates and radius of the circles
		for (x, y, r) in balls:
			# draw the circle in the output image, then draw a rectangle
			# corresponding to the center of the circle
			cv2.circle(cv_image, (x, y), r, (0, 255, 0), 4)
			#cv2.rectangle(cv_image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
	 
		# show the output image
		#cv2.imshow("output", np.hstack([image, output]))

	#find function in opencv that tells distance and size conversion from pixels to actual size

	cv2.imshow("view from turtlebot3", cv_image)
	cv2.waitKey(3)



if __name__ == '__main__':
	rospy.init_node('fetcher')

	


	turtle_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	pose_read = rospy.Subscriber('/odom', Odometry, update_odom)
	scan = rospy.Subscriber('/scan', LaserScan, handle_scan)
	camera = rospy.Subscriber('/camera/rgb/image_raw', Image, handle_image)
	
	move = Twist()
	move.linear.x = 1

	turtle_vel.publish(move)

	rate = rospy.Rate(20)
	while not rospy.is_shutdown():

		

		rate.sleep()
		#rospy.spin()