#!/usr/bin/env python
import roslib
#roslib.load_manifest('turtle_tf')
import rospy

import math
import tf
from tf.transformations import *
import turtlesim.msg
import turtlesim.srv
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import random


class State():
	def __init__(self, state, odom):
		self.myState = state
		self.myOdom = odom

PI = 3.1415926535897
laser = LaserScan()


#need to set rviz params
laser.range_min = rospy.get_param('~range_min',1)
laser.range_max = rospy.get_param('~range_max',2)
laser.angle_min = rospy.get_param('~a_min', PI)
laser.angle_max = rospy.get_param('~a_max', -PI)
laser.angle_increment = rospy.get_param('~a_inc', .01)
laser.scan_time = rospy.get_param('~scan_time', 1)

goal = (0,-5)
desiredQuat = [0,0,0,0]
state = State("toGoal", Odometry())
# turning wallnav 

def update_odom(msg):
	state.myOdom = msg
	

def handle_pose(msg, args):
	vel = args[0]
	channel = args[1]
	x=msg.pose.pose.position.x
	y=msg.pose.pose.position.y
	cmd = Twist()

	if x == 0 or y == 0 or x >11 or y > 11:
		angle = random.randint(2,4)
		cmd.angular.z = angle
		channel.publish(cmd)
		rospy.sleep(2)
		cmd = Twist()
		cmd.linear.x = vel

		channel.publish(cmd)
		rospy.sleep(1)
	else:
		cmd.linear.x = vel
		channel.publish(cmd)

def handle_scan(msg, args):
	global desiredQuat
	totalDistance = args[0]
	minDistanceWall = args[1]
	turtle_vel = args[2]
	V_explore = args[3]
	forward_ind = args[4]

	cmd = Twist()

	#if too close to a wall
	if msg.ranges[forward_ind] <= minDistanceWall and state.myState != "turning":
		turtle_vel.publish(cmd)
		rotateQuat = quaternion_from_euler(0,0,-PI/2)
		currQuat = [state.myOdom.pose.pose.orientation.x, state.myOdom.pose.pose.orientation.y, state.myOdom.pose.pose.orientation.z, state.myOdom.pose.pose.orientation.w]
		desiredQuat = quaternion_multiply(rotateQuat, currQuat)
		state.myState = "turning"
		print(euler_from_quaternion(desiredQuat))
		#print("DESIRED: "+ str(yaw))

		# cmd.angular.z = 0
		# cmd.angular.z = -PI/2
		# turtle_vel.publish(cmd)

	else:
		cmd.linear.x = V_explore*.2*msg.ranges[forward_ind]
		turtle_vel.publish(cmd)

#returns (index for forward laser scan, index for left laser scan)
def find_laser_indices():
	currQuat = [state.myOdom.pose.pose.orientation.x, state.myOdom.pose.pose.orientation.y, state.myOdom.pose.pose.orientation.z, state.myOdom.pose.pose.orientation.w]
	(roll, pitch, yaw) = euler_from_quaternion(currQuat)
	angle = laser.angle_min
	ind = 0
	while angle < laser.angle_max:
		if angle < yaw+laser.angle_increment/2 and angle > yaw-laser.angle_increment/2:
			break
		angle += laser.angle_increment
		ind += 1
	angle = laser.angle_min
	leftind = 0
	yaw = yaw-PI/2
	while angle < laser.angle_max:
		if angle < yaw+laser.angle_increment/2 and angle > yaw-laser.angle_increment/2:
			break
		angle += laser.angle_increment
		leftind += 1
	state.forwardind = ind
	state.leftind = leftind



if __name__ == '__main__':
	rospy.init_node('meriwether')

	global desiredQuat

	V_explore = rospy.get_param('~V_explore')
	totalDistance = rospy.get_param('~totalDistance')
	minDistanceWall = rospy.get_param('~minDistanceWall')

	angle = laser.angle_min
	ind = 0
	while angle < laser.angle_max:
		if angle < abs(laser.angle_increment/2):
			break
		angle += laser.angle_increment
		ind += 1


	turtle_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	pose_read = rospy.Subscriber('/odom', Odometry, update_odom)
	scan = rospy.Subscriber('/scan', LaserScan, handle_scan, callback_args = (totalDistance, minDistanceWall, turtle_vel, V_explore, ind))
	
	#set desired quaternion to the current starting quaternion
	rospy.sleep(.1)
	desiredQuat = [state.myOdom.pose.pose.orientation.x, state.myOdom.pose.pose.orientation.y, state.myOdom.pose.pose.orientation.z, state.myOdom.pose.pose.orientation.w]

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():


		currQuat = [state.myOdom.pose.pose.orientation.x, state.myOdom.pose.pose.orientation.y, state.myOdom.pose.pose.orientation.z, state.myOdom.pose.pose.orientation.w]
		(roll, pitch, currYaw) = euler_from_quaternion(currQuat)
		(roll, pitch, yaw) = euler_from_quaternion(desiredQuat)
		# print(currYaw)
		# print(yaw)
		# print()
		while(abs(currYaw - yaw) >.01):
			cmd = Twist()
			if currYaw > yaw:
				turnbit = -1
			else:
				turnbit = 1
			cmd.angular.z = turnbit*PI/2 *abs(currYaw - yaw)
			turtle_vel.publish(cmd)
			currQuat = [state.myOdom.pose.pose.orientation.x, state.myOdom.pose.pose.orientation.y, state.myOdom.pose.pose.orientation.z, state.myOdom.pose.pose.orientation.w]
			(roll, pitch, currYaw) = euler_from_quaternion(currQuat)
		state.myState = "wallnav"

		rate.sleep()
		#rospy.spin()
	