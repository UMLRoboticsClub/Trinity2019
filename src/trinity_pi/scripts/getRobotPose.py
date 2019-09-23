#!/usr/bin/env python

from trinity_pi.srv import *
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
import tf

pose = PoseStamped()

class robotPose:
	def __init__(self):
		rospy.loginfo("starting robot pose service")
		self.t = tf.TransformListener()
		s = rospy.Service('GetRobotPose', GetRobotPose, self.handle_get_robot_pose)
		rospy.Subscriber("odom", Odometry, self.callback)
		self.pub = rospy.Publisher('robot_pose', PoseStamped, queue_size=10)
	
	def callback(self, data):
		try:
			self.t.waitForTransform('/map', '/odom', rospy.Time(0), rospy.Duration(0))
			pose.pose = data.pose.pose
			pose.header = data.header
			(trans, rot) = self.t.lookupTransform('/map', '/odom', rospy.Time())
			rospy.loginfo(trans)
			rospy.loginfo(rot)
			pose.pose.position.x += trans[0]
			pose.pose.position.y += trans[1]
			pose.pose.position.z += trans[2]
			pose.pose.orientation.x += rot[0]
			pose.pose.orientation.y += rot[1]
			pose.pose.orientation.z += rot[2]
			pose.pose.orientation.w += rot[3]
			
			pose.header.frame_id = '/map'
			pose.header.stamp = rospy.Time.now()
			pose.header.seq = 0
			self.pub.publish(pose)
		except:
			pass
	
	def handle_get_robot_pose(self, req):
		return GetRobotPoseResponse(pose)

if __name__=='__main__':
	rospy.init_node('GetRobotPoseNode')
	#rospy.sleep(10)
	rp = robotPose()
	rospy.spin()



