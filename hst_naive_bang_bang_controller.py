#!/usr/bin/python
# Author Harry Terkanian
# July 16, 2017

import rospy
import math
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan


class NaieveBangBangControllerNode:


    def __init__(self):

        # state variables
        self.set_point                      = 0.25          # about 10 in
        self.correction_angle               = math.pi / 8   # about 22.5 degrees
        self.first_scan_msg                 = True
        self.minus_90_range_index           = 0
        self.error_distance                 = 0
        self.scan_range_array_center        = 0
        self.range_elements_in_90_degrees   = 0

        # subscribe to lidar scan input
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # publisher for Ackermann drive navigation topic
        self.cmd_pub = rospy.Publisher(
            "/vesc/ackermann_cmd_mux/input/navigation",
            AckermannDriveStamped, 
            queue_size = 10)

    def scan_callback(self, msg):
        # determine -90 degree index into /scan data.ranges[] (once)
        # issue steering command based on fixed correction angle 
        # and sign of error

        self.scan_msg = msg

        if (self.first_scan_msg):
            # on first scan msg, calculate portion of range to examine
            self.first_scan_msg = False                  # thru here once only
            self.scan_range_array_length = len(self.scan_msg.ranges)
                # range array length
            self.scan_range_array_center = self.scan_range_array_length // 2 
                # range array center
            # calculate number of scans in pi/4 arc
            self.range_elements_in_90_degrees = (( math.pi / 2.0 )
                // self.scan_msg.angle_increment)
            # determine beginning and end of range array to scan
            self.minus_90_range_index = int( self.scan_range_array_center 
                 - self.range_elements_in_90_degrees ) 
            print('index for -90 distance: %d' % self.minus_90_range_index) 

        self.error_distance = (self.set_point
            - msg.ranges[self.minus_90_range_index])
        self.msg = AckermannDriveStamped()
        self.msg.drive.speed = 0.5
        self.msg.drive.steering_angle = math.copysign(self.correction_angle, 
             self.error_distance)
        print('error distance: %.2f, steering_angle: %.2f'
             % (self.error_distance, self.msg.drive.steering_angle))
        if (math.copysign(self.error_distance, 1) < 0.0001):
            # compare abs value of error
            # if small; straight ahead (close enough for gove. work)
            self.msg.drive.steering_angle = 0
        self.cmd_pub.publish(self.msg)


if __name__ == "__main__":
    rospy.init_node( "hst_naive-bb_controller", anonymous = True )
    node = NaieveBangBangControllerNode()
    rospy.spin()
