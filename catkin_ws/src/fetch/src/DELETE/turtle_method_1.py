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

range_min = 0
range_max = 0
angle_min = 0
angle_max = 0
angle_increment = 0
scan_time = 0
ranges = []
def laser_poseCallback(laser_message):
    global range_min, range_max, angle_min, angle_max, angle_increment, scan_time, ranges
    range_min = laser_message.range_min
    range_max = laser_message.range_max
    angle_min = laser_message.angle_min
    angle_max = laser_message.angle_max
    angle_increment = laser_message.angle_increment
    scan_time = laser_message.scan_time
    ranges = laser_message.ranges
    loop_rate = rospy.Rate(1000000)
    loop_rate.sleep()


odomDistance = 0
odom_x = 0
odom_y = 0
odom_yaw = 0
old_odom_x = 0
old_odom_y = 0
roll = 0
pitch = 0
yaw = 0
odom_just_begin = True
odom_angular_z = 0
new_yaw = 0
def odom_poseCallback(odom_message):
    loop_rate = rospy.Rate(1000000)
    global odom_x, odom_y, old_odom_x, old_odom_y, odomDistance, odom_just_begin, roll, pitch, yaw, new_yaw, odom_angular_z
    odom_x = odom_message.pose.pose.position.x
    odom_y = odom_message.pose.pose.position.y

    odom_angular_z = odom_message.twist.twist.angular.z

    a = odom_message.pose.pose.orientation.x
    b = odom_message.pose.pose.orientation.y
    c = odom_message.pose.pose.orientation.z
    d = odom_message.pose.pose.orientation.w
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([a,b,c,d])
    if yaw < 0:
        yaw += 2 * pi
    new_yaw = yaw

    if odom_just_begin:
        old_odom_x = odom_x
        old_odom_y = odom_y
        odom_just_begin = False
    odomDistance += distance(odom_x, odom_y, old_odom_x, old_odom_y)
    old_odom_x = odom_x
    old_odom_y = odom_y
    loop_rate.sleep()



gazeboDistance = 0
gazebo_x = 0
gazebo_y = 0
old_gazebo_x = 0
old_gazebo_y = 0
gazebo_just_begin = True
def gazebo_poseCallback(gazebo_message):
    global gazebo_x, gazebo_y, old_gazebo_x, old_gazebo_y, gazeboDistance, gazebo_just_begin
    gazebo_x = gazebo_message.pose[2].position.x
    gazebo_y = gazebo_message.pose[2].position.y

    if gazebo_just_begin:
        old_gazebo_x = gazebo_x
        old_gazebo_y = gazebo_y
        gazebo_just_begin = False

    gazeboDistance += distance(gazebo_x, gazebo_y, old_gazebo_x, old_gazebo_y)
    old_gazebo_x = gazebo_x
    old_gazebo_y = gazebo_y





def find_shortest_range():
    shortest_range = 1000000
    shortest_i = 0
    for i in range(len(ranges)):
        if ranges[i] < shortest_range:
            shortest_range = ranges[i]
            shortest_i = i
    return shortest_i

def find_ninty_degree_i():
    global angles
    ninty_degree_i = 0
    shortest_range = 1000000
    for i in range(len(angles)):
        if abs(angles[i] - pi/2) < shortest_range:
            shortest_range = abs(angles[i] - pi/2)
            ninty_degree_i = i
    return ninty_degree_i

def find_longest_range():
    longest_range = 0
    longest_i = 0
    for i in range(len(ranges)):
        if ranges[i] > longest_range:
            longest_range = ranges[i]
            longest_i = i
    return longest_i
    #longest_ranges = []
    #for i in range(len(ranges)):
    #    if ranges[i] == longest_range:
    #        longest_ranges.append(i)
    #return longest_ranges[len(longest_ranges)/2]


def run_away():
    velocity_message = Twist()
    longest_i = find_longest_range()
    shortest_i = 0
    while abs(shortest_i - longest_i) > 5:
        longest_i = find_longest_range()
        loop_rate = rospy.Rate(1000)
        K = 0.01
        error = shortest_i - longest_i
        control = K * error
        velocity_message.angular.z = control
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()
    start_time = time.time()
    end_time = start_time
    print(start_time)
    print(end_time)
    while(end_time - start_time < 5):
        velocity_message.linear.x = 0.2
        velocity_publisher.publish(velocity_message)
        end_time = time.time()
    velocity_message.linear.x = 0
    velocity_publisher.publish(velocity_message)

R_message = Twist()
def turn(to_degree):
    global R_message, new_yaw
    if new_yaw < pi:
        if to_degree > new_yaw and to_degree < new_yaw + pi:
            error = to_degree - new_yaw
        else:
            if to_degree <= new_yaw:
                error = to_degree - new_yaw
            else:
                error = to_degree - 2 *pi - new_yaw
    else:
        if to_degree < new_yaw and to_degree > new_yaw - pi:
            error = to_degree - new_yaw
        else:
            if to_degree > new_yaw:
                error = to_degree - new_yaw
            else:
                error = 2 * pi - new_yaw + to_degree
    print("error = " + str(error))
    #new_yaw = yaw if yaw > 0 else yaw + 2 * math.pi
    #error = to_degree - new_yaw
    total_error = 0
    last_error = 0
    dt = 0.1
    #loop_rate = rospy.Rate(1 / dt)
    loop_rate = rospy.Rate(100)
    once_in = False

    while abs(error) >= 0.1:
        print("turning")
        once_in = True
        Kp = 0.5
        Ki = 0
        Kd = 0
        total_error += error * dt
        diff_error = (error - last_error) / dt
        control = Kp * error + Ki * total_error + Kd * diff_error
        R_message.linear.x = 0
        R_message.angular.z = control
        velocity_publisher.publish(R_message)
        last_error = error
        error = to_degree - new_yaw
        #print("control = " + str(control))
        #print("odom_angular_z = " + str(odom_angular_z))
        loop_rate.sleep()

    #print("stop")
    #R_message = Twist()
    #R_message.linear.x = 0
    #R_message.angular.z = 0
    #velocity_publisher.publish(R_message)
    #time.sleep(0.2)
    #print("stop2")


if __name__ == '__main__':
    time.sleep(1000000)
    start = time.time()
    global R_message, new_yaw
    try:
        print("TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT")
        print("A")
        time.sleep(5)
        rospy.init_node('turtlesim_motion_pose', anonymous=True)
        velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        print("B")
        laser_subscriber = rospy.Subscriber("/scan", LaserScan, laser_poseCallback)
        odom_subscriber = rospy.Subscriber("/odom", Odometry, odom_poseCallback)
        gazebo_subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, gazebo_poseCallback)
        time.sleep(2)


        print("C")
        print("range_min = " + str(range_min))
        print("range_max = " + str(range_max))
        print("angle_min = " + str(angle_min))
        print("angle_max = " + str(angle_max))
        print("angle_increment = " + str(angle_increment))
        print("ranges = " + str(ranges))



        print("D")
        totalDistance = rospy.get_param("totalDistance")
        minDistanceWall = rospy.get_param("minDistanceWall")
        target_x_pos = rospy.get_param("target_x_pos")
        target_y_pos = rospy.get_param("target_y_pos")
        #target_x_pos = 0.5
        #target_y_pos = -2
        mode = "go_straight" # go_straight, follow_wall
        #totalDistance = 30
        minDistanceWall = 0.15

        global odomDistance, gazeboDistance
        global angles
        angles = []
        for i in range(len(ranges)):
            angles.append(angle_min + i * angle_increment)
        print(angles)

        shortest_i = find_shortest_range()
        print("shortest_i = " + str(shortest_i))

        ninty_degree_i = find_ninty_degree_i()
        turn_right_i = ninty_degree_i  + 10
        turn_left_i = ninty_degree_i - 10
        chosen_i = ninty_degree_i
        print("ninty_degree_i = "+ str(ninty_degree_i))

        follow_wall = False
        #run_away()

        print(len(ranges))
        print(len(angles))

        orgin_x = odom_x
        orgin_y = odom_y


        while distance(odom_x, odom_y, target_x_pos, target_y_pos ) > 0.1:
            print("while")
            loop_rate = rospy.Rate(10)



            print("------------------------------")
            target_angle = atan2(target_y_pos - odom_y, target_x_pos - odom_x)
            if target_angle < 0:
                target_angle += 2 * pi
            print("target_angle = " + str(target_angle))
            print("yaw = " + str(yaw))

            angle_diff = int((target_angle - new_yaw) * 180 / pi)
            print(angle_diff)

            print("ranges[angle_diff] = " + str(ranges[angle_diff]))

            shortest_i = find_shortest_range()

            follow_wall = False
            for i in range(-60,60,1):
                if angle_diff + i >= 360:
                    k = angle_diff + i - 360
                elif angle_diff + i < 0:
                    k = angle_diff + i + 360
                else:
                    k = angle_diff + i
                if ranges[k] <= 1.5 * minDistanceWall:
                    follow_wall = True


            if  follow_wall :#might be a problem here
                mode = "follow_wall"
            else:
                mode = "go_straight"

            #mode = "go_straight"


            if mode == "go_straight":

                #if (abs(target_angle - new_yaw) > 0.1):
                    #R_message.linear.x = 0
                    #R_message.angular.z = 0
                    #velocity_publisher.publish(R_message)
                print("before turn")
                print("odom_x = " + str(odom_x))
                print("odom_y = " + str(odom_y))
                print("odom_angular_z = " + str(odom_angular_z))

                target_angle = atan2(target_y_pos - odom_y, target_x_pos - odom_x)
                if target_angle < 0:
                    target_angle += 2 * pi
                turn(target_angle)
                print("after_turn")
                print("odom_x = " + str(odom_x))
                print("odom_y = " + str(odom_y))
                print("odom_angular_z = " + str(odom_angular_z))
                #else:
                print("go_straight")
                print("odom_x = " + str(odom_x))
                print("odom_y = " + str(odom_y))
                print("odom_angular_z = " + str(odom_angular_z))
                print(distance(odom_x, odom_y, target_x_pos, target_y_pos))
                Kd = 0.2
                control_d = Kd * distance(odom_x, odom_y, target_x_pos, target_y_pos)
                #X_message = Twist()
                #X_message.angular.z = 0
                #X_message.linear.x = min(control_d, 0.5)
                #print(X_message.linear.x)
                #velocity_publisher.publish(X_message)
                #time.sleep(2)
                R_message.angular.z = 0.0000000000
                R_message.linear.x = max(control_d, 0.3)
                velocity_publisher.publish(R_message)
                print("after_go_straight")
                print("odom_x = " + str(odom_x))
                print("odom_y = " + str(odom_y))
                print("odom_angular_z = " + str(odom_angular_z)) #""""""

            
            elif mode == "follow_wall":
                print("follow_wall")
                shortest_i = find_shortest_range()
                print(ranges[shortest_i])
                print(minDistanceWall)
                if ranges[shortest_i] > 1.1 * minDistanceWall:
                    chosen_i = turn_left_i
                elif ranges[shortest_i] < 0.8 * minDistanceWall:
                    chosen_i = turn_right_i
                else:
                    chosen_i = ninty_degree_i

                start_time = time.time()
                while abs(shortest_i - chosen_i) > 5:
                    print("y")
                    loop_rate = rospy.Rate(10)
                    K = 0.005
                    shortest_i = find_shortest_range()
                    print("shortest_i = " + str(shortest_i))
                    print("chosen_i = " + str(chosen_i))
                    print(ranges[shortest_i])
                    error = shortest_i - chosen_i
                    control = K * error

                    R_message.linear.x = 0
                    R_message.angular.z = control
                    velocity_publisher.publish(R_message)

                    end_time = time.time()
                    if end_time - start_time > 20:
                        print(start_time)
                        print(end_time)
                        print("RUNAWAY!!!")
                        raise ()
                        run_away()
                        break
                    loop_rate.sleep()
                R_message.linear.x = 0.08
                R_message.angular.z = 0
                velocity_publisher.publish(R_message)
            loop_rate.sleep()#"""


    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")

    R_message.linear.x = 0
    velocity_publisher.publish(R_message)

    end = time.time()
    print("time = " + str(end - start))