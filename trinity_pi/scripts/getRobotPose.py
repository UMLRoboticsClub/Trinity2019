#!/usr/bin/env python

from GetRobotPose.srv import *
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
import tf

pose = PoseStamped()

if __name__=='__main__':
	rospy.init_node('GetRobotPoseNode')
	rospy.Subscriber("odom", Odometry, callback)
	s = rospy.Service('GetRobotPose', GetRobotPose, handle_get_robot_pose)
        listener = tf.TransformListener()
	rospy.spin()

def handle_get_robot_pose(req):
	return GetRobotPoseResponse(pose)

def callback(data):
        tf.waitForTransform('/map', '/odom', rospy.Time.now())
        pose = transformPose('/map', data.pose)

