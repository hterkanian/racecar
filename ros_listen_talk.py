#!/usr/bin/python

"""
Harry Sarkis Terkanian

ROS Python topic subscriber and publisher

Subscribes to std_msgs.msg of type String on topic /chatter
Publishes received messages on topic /chatter2

see: wiki.ros.org/rospy/Overview/Publishers and Subscribers
"""

import rospy
from std_msgs.msg import String

pub = rospy.Publisher('chatter2', String, queue_size = 10)

def chatter_callback(msg, pub):
    """ROS callback function republishes message to topic '/chatter2'."""
    pub.publish(msg)

rospy.Subscriber("chatter", String, chatter_callback, pub)

if __name__ == '__main__':
    rospy.init_node('hst_chatter_subscriber')
    rospy.spin()
