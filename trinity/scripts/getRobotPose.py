#!/usr/bin/env python

from GetRobotPose.srv import *
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

pose = PoseStamped()

def main():
	rospy.init_node('GetRobotPoseNode')
	rospy.Subscriber("odom", PoseWithCovarianceStamped, callback)
	s = rospy.Service('GetRobotPose', GetRobotPose, handle_get_robot_pose)
	rospy.spin()

def handle_get_robot_pose(req):
	return GetRobotPoseResponse(pose)

def callback(data):
	pose.pose = data.pose.pose
	pose.header.seq = data.header.seq
	pose.header.stamp = rospy.Time.now()
	pose.header.frame_id = data.header.frame_id

