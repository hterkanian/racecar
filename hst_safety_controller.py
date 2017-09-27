#!/usr/bin/python
#
# Author Harry Terkanian
# July 16, 2017
#
# monitor the distance to objects in front of the car; if 
# approaching minimum safe distance slow down, if at 
# minimum safe distance stop.

import rospy
import math
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan


class HSTSafetyControllerNode:


    def __init__( self ):
        """ Initialize state variables, setup ROS subscriptions and publications."""

        # state variables
        self.safe_distance  = 0.1       # about 10 cm
        self.first_scan_msg = True
        self.begin_scan_arc = 0         # right edge of area to scan
        self.end_scan_arc   = 0         # left edge of area to scan
        self.min_distance   = 0
        # subscribe to lidar scan input
        rospy.Subscriber( "/scan", LaserScan, self.scan_callback )
        # publisher for the safe Ackermann drive command
        self.cmd_pub = rospy.Publisher( "/vesc/ackermann_cmd_mux/input/safety", 
            AckermannDriveStamped, queue_size = 10 )


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
            self.sixth_arc = ((math.pi / 6.0) 
                    // self.scan_msg.angle_increment)
            # determine beginning and end of range array to scan
            self.begin_scan_arc = int(self.scan_range_array_center 
                - self.sixth_arc)   
            self.end_scan_arc   = int(self.scan_range_array_center 
                + self.sixth_arc)

        self.min_distance = min( msg.ranges[self.begin_scan_arc 
                : self.end_scan_arc])

        self.msg = AckermannDriveStamped()

        if (self.min_distance < 3 * self.safe_distance):
            self.msg.drive.speed = 0.2      # close; slow down
            self.cmd_pub.publish(self.msg)
        elif (self.min_distance < 1.5 * self.safe_distance):
            self.msg.drive.speed = 0.1      # closer yet; slow down more
            self.cmd_pub.publish(self.msg)
        elif (self.min_distance < self.safe_distance):
            self.msg.drive.speed = 0.0      # too close; stop
            self.cmd_pub.publish(self.msg)


if __name__ == "__main__":
    rospy.init_node("hst_safety_controller", anonymous = True)
    node = HSTSafetyControllerNode()
    rospy.spin()
