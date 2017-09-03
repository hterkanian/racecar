#!/usr/bin/python
# Author Harry Terkanian
# July 19, 2017

import rospy
import math
from nav_msgs.msg import Odometry


class VelocityCalculatorNode:


    def __init__(self):

        # subscribe to odometer output
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

    def odom_callback(self, msg):
        self.odom_msg = msg
        print("velocity: %.2f" 
            % math.sqrt(self.odom_msg.twist.twist.linear.x ** 2 
            + self.odom_msg.twist.twist.linear.y ** 2 ))


if __name__ == "__main__":
    rospy.init_node("hst_velocity_calculator", anonymous = True)
    node = VelocityCalculatorNode()
    rospy.spin()
