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
from math import *
import time
from std_msgs.msg import Float64

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class State():
	def __init__(self, state, odom):
		self.myState = state
		self.myOdom = odom
		self.bridge = CvBridge()


def distance(px, py, qx, qy):
	return sqrt((px - qx)**2 + (py-qy)**2)

myState = State("findBalls", Odometry())

def update_odom(msg):
	myState.myOdom = msg


ranges = []
def handle_scan(laser_message):
	global ranges
	ranges = laser_message.ranges


ball_list = []
move_list = []
yaw_list = []
target_ball_diameter = 0.2
diameter_tolerance = 0.03
distinguish_ball = 0.4
doneSearching = False


def check_ball(balls, cv_image):
	global ball_list, move_list, yaw_list
	away = 0.5
	if balls is not None:
		balls = np.round(balls[0, :]).astype("float")
		for (x, y, r) in balls:
			# If color is not red, ignore it
			if cv_image[y, x][0] != 0 or cv_image[y, x][1] != 0 or cv_image[y, x][2] != 255:
				return
			# Find the angle to face
			sign = 1 if (x -320 <= 0) else -1
			angle_rad = abs(x - 320) * (62.0 / 640) * (3.14 / 180)
			angle_idx = int(angle_rad / (6.28 / len(ranges)))
			idx = angle_idx if (x -320 <= 0) else len(ranges) - angle_idx
			if idx == len(ranges):
				idx = 0
			#print(str(str(angle_rad) + " " + str(angle_idx) + " " + str(idx)))
			#print("distance = " + str(ranges[idx]))
			odom_x = myState.myOdom.pose.pose.position.x
			odom_y = myState.myOdom.pose.pose.position.y
			if str(ranges[idx]) == "inf" or ranges[idx] > 3.0:
				return
			a = myState.myOdom.pose.pose.orientation.x
			b = myState.myOdom.pose.pose.orientation.y
			c = myState.myOdom.pose.pose.orientation.z
			d = myState.myOdom.pose.pose.orientation.w
			(roll, pitch, yaw) = tf.transformations.euler_from_quaternion([a, b, c, d])

			#print("roll = " + str(roll))
			#print("pitch = " + str(pitch))

			yaw += sign * angle_rad
			# Find the ball postition and diameter
			ball_x = odom_x + ranges[idx] * cos(yaw)
			ball_y = odom_y + ranges[idx] * sin(yaw)
			diameter = 2 * ranges[idx] * tan((r * 62.0 / 640) * (3.14 / 180))
			#print("----")
			#print(odom_x)
			#print(odom_y)
			#print("a = " + str(a))
			#print("b = " + str(b))
			#print("diameter = " + str(diameter))

			# If diameter difference is less than 3cm use it
			if abs(diameter - target_ball_diameter) > diameter_tolerance:
				return
			use = True
			for item in ball_list:
				# We don't distinguish balls within a distance (0.2m)
				if distance(ball_x, ball_y, item[0], item[1]) < distinguish_ball:
					use = False
			if use:
				goto_x = ball_x + away * cos(yaw + pi)
				goto_y = ball_y + away * sin(yaw + pi)
				ball_list.append([ball_x, ball_y])
				print("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
				move_list.append([goto_x, goto_y])
				print("YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY")
				yaw_list.append(yaw)
				print("ball_list = " + str(ball_list))
				print("move_list = " + str(move_list))



def handle_image(msg):
	try:
		cv_image = myState.bridge.imgmsg_to_cv2(msg, "bgr8")
	except CvBridgeError as e:
		print(e)
	
	(rows, cols, channels) = cv_image.shape
	gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
	balls = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1, minDist=100, param1=100, param2=30, minRadius=5, maxRadius=250)

	if not doneSearching:
		check_ball(balls, cv_image)

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







def move(target_x_pos, target_y_pos, yaw = 1.0):
	print("A")
	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	client.wait_for_server()
	goal = MoveBaseGoal()
	print("B")
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = target_x_pos
	goal.target_pose.pose.position.y = target_y_pos
	goal.target_pose.pose.orientation.w = 1.0

	# q = tf.transformations.quaternion_from_euler(0.00318615764724, 0, yaw, 'ryxz')
	# goal.target_pose.pose.orientation.x = q[0]
	# goal.target_pose.pose.orientation.y = q[1]
	# goal.target_pose.pose.orientation.z = q[2]
	# goal.target_pose.pose.orientation.w = q[3]

	client.send_goal(goal)
	wait = client.wait_for_result()
	if not wait:
		rospy.logerr("no signal")
		rospy.signal_shutdown("no signal")
	else:
		return client.get_result()

def move_manually(x , y, vel_publisher):
	odom_x = myState.myOdom.pose.pose.position.x
	odom_y = myState.myOdom.pose.pose.position.y
	target_yaw = atan2(odom_y - y, odom_x - x)
	print(target_yaw)

	currQuat = [myState.myOdom.pose.pose.orientation.x, myState.myOdom.pose.pose.orientation.y, myState.myOdom.pose.pose.orientation.z, myState.myOdom.pose.pose.orientation.w]
	(roll, pitch, currYaw) = tf.transformations.euler_from_quaternion(currQuat)
	while (abs(target_yaw - currYaw) > .01):
		cmd = Twist()
		cmd.angular.z = .6 * abs(target_yaw - currYaw) if .6 * abs(target_yaw - currYaw) < 0.6 else 0.6
		vel_publisher.publish(cmd)
		currQuat = [myState.myOdom.pose.pose.orientation.x, myState.myOdom.pose.pose.orientation.y, myState.myOdom.pose.pose.orientation.z, myState.myOdom.pose.pose.orientation.w]
		(roll, pitch, currYaw) = tf.transformations.euler_from_quaternion(currQuat)
	print("FACING AWAY FROM BALL")

	closer = .45
	while(distance(odom_x,odom_y, x, y) > closer):
		cmd = Twist()
		cmd.linear.x = -.1 * distance(odom_x,odom_y, x, y)
		vel_publisher.publish(cmd)
		odom_x = myState.myOdom.pose.pose.position.x
		odom_y = myState.myOdom.pose.pose.position.y
	print("BACKED UP")





if __name__ == '__main__':
	rospy.init_node('fetcher')
	scan = rospy.Subscriber('/scan', LaserScan, handle_scan)
	turtle_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	pose_read = rospy.Subscriber('/odom', Odometry, update_odom)
	scan = rospy.Subscriber('/scan', LaserScan, handle_scan)
	camera = rospy.Subscriber('/camera/rgb/image_raw', Image, handle_image)
	joint = rospy.Publisher('/control/base_to_arm_joint_position_controller/command', Float64)
	rate = rospy.Rate(1)
	time.sleep(7)

	try:
		# Firstly, explore the map to locate balls
		original = [-2.0, 3.5]
		#points = [[1.8, -1]] # Inside
		#points = [[3.5, 2.0]] # First point
		#points = [[1.1, 3.5]] # Near point

		points = [[3.5, 2.0]] # t1 t2 t4 t6 t7
		#points = [[0, -3.5]] # t3
		#points = [[1.8, -1]] # t5


		for i in range(len(points)):
			print("---------------------------------")
			print("i = " + str(i))
			print("point = " + str(points[i]))
			result = move(points[i][0], points[i][1])#, pi/2)

		doneSearching = True

		my_ball_list = ball_list
		my_move_list = move_list
		my_yaw_list = yaw_list
		print("=================================================")
		print("There are {} balls".format(len(my_ball_list)))
		print("my_ball_list = " + str(my_ball_list))
		print("my_move_list = " + str(my_move_list))
		print("=================================================")

		for i in range(len(my_ball_list) - 1, -1, -1):
			print("GO TO NEXT POINT : " + str(my_move_list[i]))
			time.sleep(3)
			result = move(my_move_list[i][0], my_move_list[i][1])#, my_yaw_list[i] + pi)
			## Get the ball
			## TO DO
			move_manually(my_ball_list[i][0], my_ball_list[i][1], turtle_vel)

			#catch the ball
			joint.publish(Float64(-1.57))


			#time.sleep(3)
			print("GO HOME ")
			result = move(original[0], original[1])
			#release the ball
			joint.publish(Float64(0))
			time.sleep(3)

	except rospy.ROSInterruptException:
		rospy.loginfo("navigation interrupted.")

	while not rospy.is_shutdown():
		rate.sleep()
		#rospy.spin()