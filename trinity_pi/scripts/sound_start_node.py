#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

import frequency_detect.py

node_name = 'sound_start_node'
topic_name = 'sound_start'

if __name__ == '__main__':
    rospy.init_node(node_name)
    pub = rospy.Publisher(topic_name, Bool, queue_size=1)

    wait_for_freq()

    pub.publish(True)
    rospy.loginfo("Frequency found")
