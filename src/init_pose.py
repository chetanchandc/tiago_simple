#!/usr/bin/env python

import rospy 
from geometry_msgs.msg import PoseWithCovarianceStamped

rospy.init_node("set_init_pose", anonymous=True)
pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)

def initpose_msg(msg):
        initpose_msg = PoseWithCovarianceStamped()
	intipose_msg.header.frame_id = "map"
	initpose_msg.pose.pose.position.x = 0.0
	initpose_msg.pose.pose.position.y = 0.0
	initpose_msg.pose.pose.position.z = 0.0
	initpose_msg.pose.pose.orientation.x = 0.0
	initpose_msg.pose.pose.orientation.y = 0.0
	initpose_msg.pose.pose.orientation.z = 0.05
	initpose_msg.pose.pose.orientation.w = 0.9

rospy.sleep(1)

rospy.loginfo("Setting Init_pose")
pub.publish(initpose_msg)
rospy.loginfo("initial pose set")
