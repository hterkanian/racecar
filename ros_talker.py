#!/usr/bin/python

"""
Harry Sarkis Terkanian

ROS Python topic publisher

Publishes std_msgs.msg of type String to topic /chatter at rate of 10hz

see: wiki.ros.org/rospy/Overview/Publishers and Subscribers

"""

import rospy
from std_msgs.msg import String

rospy.init_node('hst_chatter_publisher')

pub = rospy.Publisher('chatter', String, queue_size = 10)

timer = rospy.Rate(10)

while not rospy.is_shutdown():
	pub.publish('Danger Will Robinson!')
	timer.sleep()
