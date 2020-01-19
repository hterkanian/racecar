#!/usr/bin/python
"""
    Author Harry Terkanian

    December 18, 2018
    Most recent revision: January 22, 2019

    Data collection script.  Based on safety_controller.py
    collects timestamped velocity and relative position from /odom topic.
    collects timestamped distance to object in front of car from /scan topic.
    pickles both data structures on receipt of termination signal in control 
    file. 

    Terminate script by issuing "echo 1 > control from console
"""

import rospy
import math
import pickle
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class SafetyDataCollectorNode:


    def __init__( self ):
        """
        Initialize state variables, setup ROS subscriptions and publications, 
        initializes control file.
        """

        # state variables
        self.odom_time      = 0.00
        self.odom_x_pos     = 0.00
        self.odom_y_pos     = 0.00
        self.odom_velocity  = 0.00
        self.odom_dict      = {}    # single odom entry
        self.odom_list      = []    # list of self.odom_dict entries

        self.scan_distance  = 0.00
        self.scan_time      = 0.00
        self.scan_dict      = {}    # single scan entry
        self.scan_list      = []    # list of self.scan_dict entries

        self.debug_switch   = False
        self.shutdown_flag  = True  #one time flag to avoid race condition

        # subscribe to lidar scan and odometry messages
        self.sub_scan = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        #initialize control file with "0"
        self.control_file = open("control", "w")
        self.control_file.write("0")
        self.control_file.close()
        self.control_file = open("control", "r")


    def check_for_shutdown(self):
        """
        Checks control file for shutdown signal; initiates orderly shutdown.
        """

        if self.shutdown_flag:
            exit_flag = self.control_file.read()
            if exit_flag != "0":    # terminate on anything but "0" in control file
                self.shutdown_flag = False
                print("\n" + str(len(self.scan_list)) + " scan records")
                print(str(len(self.odom_list)) + " odom records")
                with open("scan_data", "wb") as sd:
                    pickle.dump(self.scan_list, sd)
                with open("odom_data", "wb") as so:
                    pickle.dump(self.odom_list, so)
                self.sub_scan.unregister()
                self.sub_odom.unregister()
                rospy.signal_shutdown("Exiting")
            self.control_file.seek(0)

    def odom_callback(self, msg):       # how fast are we going?
        """
        Collects timestamped calculated velocity and relative (x, y) position.
        """

        self.odom_msg = msg
        self.odom_velocity =  1.0 * math.sqrt(self.odom_msg.twist.twist.linear.x ** 2 +
                    self.odom_msg.twist.twist.linear.y ** 2)

        self.odom_x_pos = self.odom_msg.pose.pose.position.x
        self.odom_y_pos = self.odom_msg.pose.pose.position.y
        secs = self.odom_msg.header.stamp.secs
        nsecs = self.odom_msg.header.stamp.nsecs
        self.odom_time = float(secs) + float(nsecs) / 10 ** 9
        self.odom_dict = {
                'time': self.odom_time, 
                'velocity': self.odom_velocity, 
                'x': self.odom_x_pos, 
                'y': self.odom_y_pos
                }
        self.odom_list.append(self.odom_dict)

        self.check_for_shutdown()

        if self.debug_switch:
            print("odom time: " + str(self.odom_time) + 
                    " velocity: " + str(self.odom_velocity) + 
                    " x coord: " + str(self.odom_x_pos) + 
                    " y coord: " + str(self.odom_y_pos) + 
                    '\n'
                    )

    def scan_callback(self, msg):
        """
        Collects timestamped straight ahead distance from /scan topic
        """

        self.scan_msg = msg
        secs = self.scan_msg.header.stamp.secs
        nsecs = self.scan_msg.header.stamp.nsecs
        self.scan_time = float(secs) + float(nsecs) / 10 ** 9
        self.scan_distance = self.scan_msg.ranges[540]
        self.scan_dict = {'time': self.scan_time, 'distance': self.scan_distance}
        self.scan_list.append(self.scan_dict)

        self.check_for_shutdown()

        if self.debug_switch:
            print("scan time: " + str(self.scan_time) + 
                    " distance: " + str(self.scan_distance) + 
                    '\n'
                    )

if __name__ == "__main__":
    rospy.init_node("safety_controller", anonymous = True)
    node = SafetyDataCollectorNode()
    rospy.spin()
    print("Safety data capture terminated.")
