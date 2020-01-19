#!/usr/bin/python

"""
Harry Sarkis Terkanian

ROS Python topic subscriber

Subscribes to std_msgs.msg of type String on topic /chatter

see: wiki.ros.org/rospy/Overview/Publishers and Subscribers
"""

import rospy
from std_msgs.msg import String

def chatter_callback(msg):
    """ROS callback function prints message text to console."""
    print(msg)

rospy.Subscriber("chatter", String, chatter_callback)

if __name__ == '__main__':
    rospy.init_node('hst_chatter_subscriber')
    rospy.spin()
