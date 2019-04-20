#!/usr/bin/env python

import rospy
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan, Image
from math import sqrt




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




def move(target_x_pos, target_y_pos):
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

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("no signal")
        rospy.signal_shutdown("no signal")
    else:
        return client.get_result()

if __name__ == '__main__':
    global ranges
    #rospy.init_node('move')

    #laser_subscriber = rospy.Subscriber("/scan", LaserScan, laser_poseCallback)
    time.sleep(7)
    time.sleep(100000000)


    try:
        rospy.init_node('movebase_client_py')
        original = [-2.0, 3.5]
        # oct map #points = [[-1.5, -1.5],[0.5, -1.5],[1.5, 1.3],[1.5, -0.5],[-1.5, -0.5]]
        #points = [[-0.5, -0.5],[-2.5, -1.5],[-2.5, 3.5],[-2.0, -2.0]]
        #points = [[1.5, -1], [-2.5, 3.5]]
        #points = [[1.0, 3.5]]#,[1.5, -1], [-2.5, 3.5]]
        points = [[1.7, -1]]
        #points = [[3.5, 2]]
        for i in range(len(points)):
            print("---------------------------------")
            print("i = " + str(i))
            print("point = " + str(points[i]))
            result = move(points[i][0],points[i][1])
            #print("now go home: " + str(original))
            #result = move(original[0], original[1])

        my_ball_list = []#fetcher.ball_list
        my_move_list = []#fetcher.move_list
        print("=================================================")
        print("my_ball_list = " + str(my_ball_list))
        print("my_move_list = " + str(my_move_list))
        print("=================================================")
        for i in range(len(my_ball_list)-1, -1, -1):
            print("GO TO NEXT POINT : " + str(my_move_list[i]))
            time.sleep(3)
            result = move(my_move_list[i][0], my_move_list[i][1])
            ## Trap the ball
            time.sleep(3)
            print("GO HOME ")
            result = move(original[0], original[1])

    except rospy.ROSInterruptException:
        rospy.loginfo("navigation interrupted.")






