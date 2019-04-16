#!/usr/bin/env python
import roslib
roslib.load_manifest('turtle_tf')
import rospy

import tf
import turtlesim.msg
from nav_msgs.msg import Odometry

def handle_turtle_pose(msg, turtlename):
	br = tf.TransformBroadcaster()
	br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
						msg.pose.pose.orientation,
						rospy.Time.now(),
						turtlename,
						"world")

if __name__ == '__main__':
	rospy.init_node('broadcast')
	turtlename = rospy.get_param('~turtle')
	rospy.Subscriber('/odom',
	Odometry,
	handle_turtle_pose,
	turtlename)
	rospy.spin()