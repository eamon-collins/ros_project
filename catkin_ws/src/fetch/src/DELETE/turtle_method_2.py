#!/usr/bin/env python

import rospy
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def move():
    print("A")
    target_x_pos = rospy.get_param("target_x_pos")
    target_y_pos = rospy.get_param("target_y_pos")

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
    start = time.time()

    try:
        rospy.init_node('movebase_client_py')
        result = move()
        if result:
            end = time.time()
            print("time = " + str(end - start))
            rospy.loginfo("Goal is reached")
    except rospy.ROSInterruptException:
        rospy.loginfo("navigation interrupted.")



