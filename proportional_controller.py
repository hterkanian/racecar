#!/usr/bin/python
# Author Harry Terkanian
# July 19, 2017

import rospy
import math
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class HSTProportionalControllerNode:


    def __init__( self ):
        # ==============state class attributes (variables)======================
        self.set_point              = 0.75      # about 10 inches
        self.kp                     = 0.1       # proportional gain (arbitrary)
        self.kd                     = 0.1       # derivative gain (aarbitrary)
        self.drive_speed            = 0.2       # arbitrary
        self.steering_saturation    = 0.30      # max steering angle
        self.first_scan_msg         = True      # flag for self.first_scan

        # list ranges for various scan indicies (calculated by self.first_scan()
        self.minus_90_range_index           = 0
        self.plus_90_range_index            = 0
        self.minus_60_range_index           = 0
        self.plus_60_range_index            = 0
        self.scan_range_array_center        = 0
        self.range_elements_in_90_degrees   = 0
        self.range_elements_in_60_degrees   = 0

        # error
        self.error_distance     = 0
        self.error_derivative   = 0     # not implemented; implement for PD controller

        # velocity
        self.velocity           = 0     # not used; future enhancement

        # ==============ROS topic subscriptions & publications=================
        # ==============subscribe to lidar /scan input=========================
        rospy.Subscriber( "/scan", LaserScan, self.scan_callback )

        # ==============subscribe to odometer /odom output=====================
        rospy.Subscriber( "/odom", Odometry, self.odom_callback )

        # ==============publisher for Ackermann drive /navigation topic========
        self.cmd_pub = rospy.Publisher( 
                        "/vesc/ackermann_cmd_mux/input/navigation", 
                        AckermannDriveStamped, queue_size = 10)

    # ==================class method definitions (functions)===================
    def odom_callback(self, msg):
        # callback method for /odom topic
        self.odom_msg = msg         # copy /odom message to local attribute
        self.velocity = math.sqrt(self.odom_msg.twist.twist.linear.x ** 2
                        + self.odom_msg.twist.twist.linear.y ** 2)
        print("velocity: %.2f" % self.velocity)


    def scan_callback(self, msg):
        # callback method for /scan topic
        self.scan_msg = msg         # copy /scan message to local attribute

        # determine various indicies into /scan data.ranges[] (once)
        if (self.first_scan_msg):
            self.first_scan_msg = self.first_scan()

        self.error_distance = self.calculate_error_distance()
        # print("error: %.2f, slope: %.2f, intercept: %.2f"
        # % (self.error_distance, self.line_fit[1], self.line_fit[0]))

        self.error_derivative = self.calculate_error_derivative()

        self.controller()   #issue the AckermannDriveStamped message


    def controller(self):
        self.msg = AckermannDriveStamped()

        self.msg.drive.speed = self.drive_speed
        self.angle_toward_wall = -math.tan(self.line_fit[1])
            # current bearing in radians

        self.msg.drive.steering_angle = (-self.kp * self.error_distance 
            + self.kd * self.error_derivative)
        if (math.copysign(self.msg.drive.steering_angle, 1) 
            > self.steering_saturation):
            self.msg.drive.steering_angle = (self.steering-saturation 
                * math.copysign( 1, self.msg.drive.steering_angle))
        # print('error distance: %.2f, speed: %.2f steering_angle: %.2f' % 
        #       (self.error_distance, self.msg.drive.speed, 
        #       self.msg.drive.steering_angle))
        self.cmd_pub.publish(self.msg)
        

    def first_scan(self):
        # called once by scan_callback on first scan msg
        # calculates useful indicies into the /scan range[] and 
        # sets values of class attributes
        self.scan_range_array_length = len(self.scan_msg.ranges)
            # length of range list
        self.scan_range_array_center = (self.scan_range_array_length 
            // 2)    # center of range list
        # calculate number of scan sweeps in 60 & 90 degree arcs
        self.range_elements_in_90_degrees = ((math.pi / 2.0) 
            // self.scan_msg.angle_increment)
        self.range_elements_in_60_degrees = ((math.pi / 3.0) 
            // self.scan_msg.angle_increment)
        # determine indicies range list for desired agnles from center; 
        # set attribute values
        self.minus_90_range_index = int(self.scan_range_array_center 
            - self.range_elements_in_90_degrees) 
        self.plus_90_range_index  = int(self.scan_range_array_center 
             + self.range_elements_in_90_degrees) 
        self.minus_60_range_index = int(self.scan_range_array_center 
             - self.range_elements_in_60_degrees)
        self.plus_60_range_index  = int(self.scan_range_array_center 
             + self.range_elements_in_60_degrees)
        # debugging; coment out for production use
        print('ranges index for -90 degrees: %d' % self.minus_90_range_index)
        print('ranges index for +90 degrees: %d' % self.plus_90_range_index)
        print('ranges index for -60 degrees: %d' % self.minus_60_range_index)
        print('ranges index for +60 degrees: %d' % self.plus_60_range_index)
        return False        # set the flag so we don't come thru here again


    def calculate_cartesian_coords(self, element):
        # returns (x, y) coords for list element at ranges[index]
        self.distance = self.scan_msg.ranges[element]
        self.angle    = self.scan_msg.angle_increment * (element 
                          - self.scan_range_array_center)
        self.y        = self.distance * math.sin(self.angle)
        self.x        = self.distance * math.cos(self.angle)
        return (self.x, self.y)
        

    def calculate_error_distance(self):
        # interprets /scan data to identify line of wall 
        # and distance from car to wall
        # clear the x & y arrays
        self.x_array = []
        self.y_array = []
        for element in range(self.plus_60_range_index, 
            self.plus_90_range_index ):
            # ignore distances beyond lidar range
            if (self.scan_msg.ranges[element] < self.scan_msg.range_max):
                self.result = self.calculate_cartesian_coords( element)
                self.x_array.append( self.result[0])
                self.y_array.append( self.result[1])

        # fit points to first order polynomial (line)
        self.line_fit_raw = np.polyfit(self.x_array, self.y_array, 1)
        self.line_fit = np.poly1d(self.line_fit_raw)
            #  returns ( intercept, slope )
        # print( self.line_fit )
        # print("slope: %.2d; intercept: %.2d" 
        #     % (self.line_fit[0], self.line_fit[1]))
        self.shortest_distance = math.copysign(self.line_fit[0] 
            / math.sqrt(1 + self.line_fit[1] ** 2), 1)
        # print("shortest distance: %.2f" % self.shortest_distance)
        return self.set_point - self.shortest_distance


    def calculate_error_derivative(self):
        # not implemented; return zero
        return 0


if __name__ == "__main__":
    rospy.init_node( "hst_proportional_controller", anonymous = True )
    node = HSTProportionalControllerNode()
    rospy.spin()
