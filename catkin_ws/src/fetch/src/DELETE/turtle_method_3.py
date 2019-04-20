#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf


import time
from math import *

def distance(px, py, qx, qy):
    return sqrt((px - qx)**2 + (py-qy)**2)

odomDistance = 0
odom_x = 0
odom_y = 0
odom_yaw = 0
roll = 0
pitch = 0
yaw = 0
odom_just_begin = True
odom_angular_x = 0
odom_angular_y = 0
odom_angular_z = 0
odom_linear_x = 0
odom_linear_y = 0
odom_linear_z = 0
new_yaw = 0
def odom_poseCallback(odom_message):
    #loop_rate = rospy.Rate(1000000)
    global odom_x, odom_y, odomDistance, odom_just_begin, roll, pitch, yaw, new_yaw, odom_angular_z
    global odom_angular_x, odom_angular_y
    global odom_linear_x, odom_linear_y, odom_linear_z

    odom_x = odom_message.pose.pose.position.x
    odom_y = odom_message.pose.pose.position.y

    odom_angular_x = odom_message.twist.twist.angular.x
    odom_angular_y = odom_message.twist.twist.angular.y
    odom_angular_z = odom_message.twist.twist.angular.z

    odom_linear_x = odom_message.twist.twist.linear.x
    odom_linear_y = odom_message.twist.twist.linear.y
    odom_linear_z = odom_message.twist.twist.linear.z

    a = odom_message.pose.pose.orientation.x
    b = odom_message.pose.pose.orientation.y
    c = odom_message.pose.pose.orientation.z
    d = odom_message.pose.pose.orientation.w

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([a,b,c,d])
    if yaw < 0:
        yaw += 2 * pi
    new_yaw = yaw
    #loop_rate.sleep()



if __name__ == '__main__':
    rospy.init_node('turtlesim_motion_pose', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    odom_subscriber = rospy.Subscriber("/odom", Odometry, odom_poseCallback)
    time.sleep(5)

    # 1. turn to the target degree
    to_degree = 3 * pi/2
    error = to_degree - new_yaw
    while abs(error) >= 0.001:
        print("turning")
        Kp = 0.5
        control = Kp * error if Kp * error < 0.2 else 0.2
        velocity_message = Twist()
        velocity_message.linear.x = 0.0
        velocity_message.angular.z = control
        velocity_publisher.publish(velocity_message)
        error = to_degree - new_yaw
        print("pre odom_linear_x = " + str(odom_linear_x))
        print("pre odom_linear_y = " + str(odom_linear_y))
        print("pre odom_linear_z = " + str(odom_linear_z))
        print("pre odom_angular_x = " + str(odom_angular_x))
        print("pre odom_angular_y = " + str(odom_angular_y))
        print("pre odom_angular_z = " + str(odom_angular_z))
    """velocity_message = Twist()
    velocity_message.linear.x = 0.0
    velocity_message.linear.y = 0.0
    velocity_message.linear.z = 0.0
    velocity_message.angular.x = 0.0
    velocity_message.angular.y = 0.0
    velocity_message.angular.z = 0.0
    velocity_publisher.publish(velocity_message)
    rospy.sleep(5)"""

    # 2. go straight <- PROBLEM HERE!!
    """print("before sending signal to go straight")
    print("pre odom_angular_z = " + str(odom_angular_z))
    velocity_message = Twist()
    velocity_message.linear.x = 3.0
    velocity_message.angular.z = 0.0
    velocity_publisher.publish(velocity_message)
    print("after sending signal to go straight")"""

    print("pre odom_angular_z = " + str(odom_angular_z))
    print(" odom_linear_x = " + str(odom_linear_x))
    velocity_message_l = Twist()
    r = rospy.Rate(1)
    t_x = 0.5
    t_y = -2.0
    while True:
        Kr = 0.2
        control_l = Kr / distance(t_x, t_y, odom_x, odom_y)
        velocity_message_l.linear.x = 0.3#control_l
        # New line to compensate for small corrections on angular errors
        velocity_message_l.angular.z = 0.5 * (to_degree - new_yaw)
        velocity_publisher.publish(velocity_message_l)
        print(" odom_angular_z = " + str(odom_angular_z))
        print("     odom_linear_x = " + str(odom_linear_x))


    """
    for i in range(20):
        print("post odom_linear_x = " + str(odom_linear_x))
        print("post odom_linear_y = " + str(odom_linear_y))
        print("post odom_linear_z = " + str(odom_linear_z))
        print("post odom_angular_x = " + str(odom_angular_x))
        print("post odom_angular_y = " + str(odom_angular_y))
        print("post odom_angular_z = " + str(odom_angular_z))"""

    rospy.sleep(15)