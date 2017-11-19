#!/usr/bin/python
#
# Author Harry Terkanian
# July 16, 2017
#
# most recent revision November 19, 2017
#
# safety controller.
#
# monitor the distance to objects in front of the car; if 
# approaching minimum safe distance slow down, if at 
# minimum safe distance stop.
#
# minimum safe distance is set by  self.safe_distance
#
# at 3 x  minumum safe distance speed reduced from self.safe_speed to 0.0  linearly as the
# minumum distance approaches the minimum safe distance
# at minimum safe distance speed is set to 0.0
#
# possible enhancements: (1) change the arc being scanned for minimum distance 
# based on the  current minimum distance since the car is 30 cm wide, 
# (don't want to have a wheel clip an edge. . .);
# and (2) monitor the steering angle to see whether to look one side of center 
# as the car is turning. 
#

import rospy
import math
import socket
import re
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class HSTSafetyControllerNode:


    def __init__( self ):
        """ Initialize state variables, setup ROS subscriptions and publications."""

        # state variables
        self.safe_distance  = 0.10      # about 10 cm
    	self.lidar_offset   = 0.15	    # Lidar 15 cm back from front bumper
        self.safe_speed     = 0.20      # arbitrary
        self.slowdown_flag  = False     # slowdown switch
        self.first_scan_msg = True      # first scan switch
        self.begin_scan_arc = 0         # right edge of area to scan
        self.end_scan_arc   = 0         # left edge of area to scan
        self.min_distance   = 0.0       # measured minimum distance
        self.vesc           = ''        # prepend ackermann msg with /vesc?
        self.half_safety_arc =  math.pi / 6.0
        self.nav_message    = AckermannDriveStamped()
        self.safety_msg     = AckermannDriveStamped()
        self.odom_msg       = Odometry()
        self.odom_velocity  = 0.00
        self.debug_switch   = True

    	self.min_safe_distance = self.safe_distance + self.lidar_offset

        ww = re.compile('ww')
        m = ww.findall(socket.gethostname())
        if m:
            vesc = '/vesc'      # running on a laptop name begins with 'ww"
        else:
            vesc = ''           # running on a racecar

        # subscribe to lidar scan, odometry  and navigation input messages
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber(vesc + "/ackermann_cmd_mux/input/navigation", 
                AckermannDriveStamped, self.nav_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        # publisher for the safety Ackermann drive command messages
        self.cmd_pub = rospy.Publisher( vesc + "/ackermann_cmd_mux/input/safety", 
                AckermannDriveStamped, queue_size = 10 )


    def  nav_callback(self, msg):       # save the current navigation input
        self.nav_message = msg


    def odom_callback(self, msg):       # how fast are we going?
        self.odom_msg = msg
        self.odom_velocity = math.sqrt(self.odom_msg.twist.twist.linear.x ** 2 +
                    self.odom_msg.twist.twist.linear.y ** 2)


    def scan_callback(self, msg):
        """ check for min_distance less than min safe distance
        if close publish reduced speed; if at min safe distance publish 
        zero speed, both to /safety."""

        self.scan_msg = msg

        if (self.first_scan_msg):
            # first scan msg, calculate arc to scan
            self.first_scan_msg = False             # thru here once only
            self.scan_range_array_length = len(self.scan_msg.ranges)
                # range array length
            self.scan_range_array_center = (self.scan_range_array_length 
                    // 2)                           # range array center
            # calculate number of scans in pi/6 arc
            # to check pi/6 on either side of dead ahead
            self.sixth_arc = self.half_safety_arc // self.scan_msg.angle_increment
            # determine beginning and end of range array to scan
            self.begin_scan_arc = int(self.scan_range_array_center 
                - self.sixth_arc)
            self.end_scan_arc   = int(self.scan_range_array_center 
                + self.sixth_arc)

        self.min_distance = min( self.scan_msg.ranges[self.begin_scan_arc 
                : self.end_scan_arc])

        if (self.min_distance < 3.0 * self.min_safe_distance):  # getting close; slow
            self.safety_msg.drive.speed = (self.safe_speed 
		    * (self.min_distance -  self.min_safe_distance) 
		    / (6.0 * self.min_safe_distance)) 
            self.slowdown_flag = True
        if (self.min_distance < self.min_safe_distance):
            self.safety_msg.drive.speed = 0.0                   # too close; stop
            self.slowdown_flag = True
        if self.slowdown_flag:
            self.slowdown_flag = False                          #reset switch
            if (self.odom_velocity > self.safety_msg.drive.speed):
                self.cmd_pub.publish(self.safety_msg)
        if self.debug_switch:
            print("distance: %.2f; nav speed: %.2f; odom speed: %.2f; safety speed: %.2f" % 
                (self.min_distance, 
                self.nav_message.drive.speed, 
                self.odom_velocity,
                self.safety_msg.drive.speed))


if __name__ == "__main__":
    rospy.init_node("hst_safety_controller", anonymous = True)
    node = HSTSafetyControllerNode()
    rospy.spin()
