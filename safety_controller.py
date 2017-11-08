#!/usr/bin/python
#
# Author Harry Terkanian
# July 16, 2017
#
# most recent revision October 21, 2017
#
# proportional controller.
#
# monitor the distance to objects in front of the car; if 
# approaching minimum safe distance slow down, if at 
# minimum safe distance stop.
#
# minimum safe distance is set by  self.safe_distance
#
# at 2 x  minumum safe distance  speed reduced from 0.2 to 0.0  linearly as the
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
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan


class HSTSafetyControllerNode:


    def __init__( self ):
        """ Initialize state variables, setup ROS subscriptions and publications."""

        # state variables
        self.safe_distance  = 0.10      # about 10 cm
    	self.lidar_offset   = 0.15	    # Lidar 15 cm back from front bumper
    	self.min_safe_distance = self.safe_distance + self.lidar_offset
        self.safe_speed     = 0.20      # arbitrary
        self.first_scan_msg = True
        self.begin_scan_arc = 0         # right edge of area to scan
        self.end_scan_arc   = 0         # left edge of area to scan
        self.min_distance   = 0.0       # measured minimum distance
        self.half_safety_arc =  math.pi / 6.0
        self.nav_message    = AckermannDriveStamped() 
        self.safety_msg     = AckermannDriveStamped()
        # subscribe to lidar scan input
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber("/vesc/ackermann_cmd_mux/input/navigation", 
                AckermannDriveStamped, self.nav_callback)
        # publisher for the safe Ackermann drive command
        self.cmd_pub = rospy.Publisher( "/vesc/ackermann_cmd_mux/input/safety", 
            AckermannDriveStamped, queue_size = 10 )


    def  nav_callback(self, msg):
        self.nav_message = msg


    def scan_callback(self, msg):
        """ check for min_distance less than min safe distance
        if close reduce speed; if at min safe distance publish 
        to /safety with zero speed."""

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

        if (self.min_distance < self.min_safe_distance):
            self.safety_msg.drive.speed = 0.0                       # too close; stop
            self.cmd_pub.publish(self.safety_msg)
        elif (self.min_distance < 2.0 * self.min_safe_distance):
            self.slowdown_speed = (self.safe_speed 
		    * (self.min_distance -  self.min_safe_distance) 
		    / (1.0 * self.min_safe_distance)) 
            if (self.nav_message.drive.speed > self.slowdown_speed):
                self.safety_msg.drive.speed = self.slowdown_speed   # close; slow
                self.cmd_pub.publish(self.safety_msg)
        print("distance: %.2f; navigation speed: %.2f; safety speed: %.2f" % 
                (self.min_distance, 
                self.nav_message.drive.speed, 
                self.safety_msg.drive.speed))

if __name__ == "__main__":
    rospy.init_node("hst_safety_controller", anonymous = True)
    node = HSTSafetyControllerNode()
    rospy.spin()
