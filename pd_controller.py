#!/usr/bin/python
# Author Harry Terkanian
# August 27, 2017
# Most recent revision October 11, 2017
#
# currently calculates values for following either wall but only follows left wall
#

import rospy
import math
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan


class PDControllerNode:


    def __init__(self):

        # ==============state class attributes ================================
        self.set_point              = 0.50     # lidar offset is 15 cm so 35 cm 
        self.kp                     = 1.00     # proportional gain (arbitrary)
        self.kd                     = 1.00     # derivative gain (aarbitrary)
        self.drive_speed            = 0.5      # arbitrary
        self.steering_saturation    = 0.30     # max steering angle
        self.first_scan_msg         = True     # flag for self.first_scan()
        self.debug_switch           = True     # Turns on debug print statements
        # list ranges for various scan indicies 
        # (calculated by self.do_first_scan()
        self.minus_90_range_index           = 0     # 90 degrees right of center
        self.plus_90_range_index            = 0     # 90 degrees left of center
        self.minus_60_range_index           = 0     # 60 right
        self.plus_60_range_index            = 0     # 60 left
        self.center_range_index             = 0
        self.range_elements_in_90_degrees   = 0
        self.range_elements_in_60_degrees   = 0

        # velocity
        self.velocity                       = 0

        # improved error
        self.left_error_distance            = 0
        self.left_error_slope               = 0
        self.left_error_derivative          = 0
        self.left_error_list                = []
        self.left_error_times               = []

        self.right_error_distance           = 0
        self.right_error_slope              = 0
        self.right_error_derivative         = 0
        self.right_error_list               = []
        self.right_error_times              = []

        # ==========ROS topic subscriptions & publications=====================
        # ==========subscribe to lidar /scan input=============================
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        # ==========publisher for Ackermann drive /navigation topic============
        self.cmd_pub = rospy.Publisher( 
                        "/vesc/ackermann_cmd_mux/input/navigation", 
                        AckermannDriveStamped, queue_size = 10)


    # ==================class method definitions ==============================


    def scan_callback(self, msg):
        """Callback method for /scan topic.  Calculates error distance and derivative
        for both left and righ sides.  Issues appropriate instructions to drive to
        maintain desired wall separation."""

        self.scan_msg = msg

        # determine various indicies into /scan data.ranges[] (once)
        if (self.first_scan_msg):
            self.first_scan_msg = self.first_scan()

        # determine error
        left_error = self.calculate_left_error()
        #returns [left error, slope of left wall]
        self.left_error_distance = left_error[0]
        self.left_error_slope = left_error[1]
        right_error = self.calculate_right_error()
        #returns [right_error, slope of right wall]
        self.right_error_distance = right_error[0]
        self.right_error_slope = right_error[1]
        if self.debug_switch:
            print("left error: %.2f, slope: %.2f" %
                    (self.left_error_distance, self.left_error_slope))
            print("right error: %.2f, slope: %.2f" %
                    (self.right_error_distance, self.right_error_slope))

        # determine first derivative of left error using velocity and wall angle
        self.left_error_derivative = self.approx_error_derivative(
                self.left_error_distance, 
                "left")
        self.right_error_derivative = self.approx_error_derivative(
                self.right_error_distance, 
                "right")
        if self.debug_switch:
            print("left error distance: %.2f" % self.left_error_distance)
            print("left error derivative: %.2f" % self.left_error_derivative)
            print("right error distance: %.2f" % self.right_error_distance)
            print("right error derivative: %.2f" % self.right_error_derivative)

        #issue the AckermannDriveStamped message
        self.controller()


    # ==================class helper method definitions (functions)=============


    def controller(self):
        self.cmd_msg = AckermannDriveStamped()
        self.cmd_msg.drive.speed = self.drive_speed

        # calculate steering angle = Kp * e + kd * de/dt
        self.cmd_msg.drive.steering_angle = (self.kp * self.left_error_distance
                + self.kd * self.left_error_derivative)
        if self.debug_switch:
            print("left error: %.2f, steering gain: %.2f, steering angle: %.2f"
                    % (self.left_error_distance,
                    self.kp,
                    self.cmd_msg.drive.steering_angle))
            print("left error derivative: %.2f, kd: %.2d" %
                    (self.left_error_derivative, self.kd))

        # cap steering angle at saturation value
        if (abs( self.cmd_msg.drive.steering_angle) > self.steering_saturation):
            self.cmd_msg.drive.steering_angle = \
                    self.steering_saturation\
                    * math.copysign( 1, self.cmd_msg.drive.steering_angle )
        if self.debug_switch:
            print('left error distance: %.2f, derivative: %.2f steering_angle: %.2f' 
                    % (self.left_error_distance, 
                    self.left_error_derivative, 
                    self.cmd_msg.drive.steering_angle))

        self.cmd_pub.publish(self.cmd_msg)


    def first_scan(self):
        """called once by scan_callback on first scan msg; calculates useful
        indicies into the /scan range[] and saves values as class attributes."""

        self.scan_range_array_length = len(self.scan_msg.ranges)
        self.scan_center_range_index = self.scan_range_array_length // 2

        # calculate number of scan sweeps in 60 & 90 degree arcs
        self.range_elements_in_90_degrees = (math.pi / 2.0)\
                // self.scan_msg.angle_increment
        self.range_elements_in_60_degrees = (math.pi / 3.0)\
                // self.scan_msg.angle_increment
        # determine indicies range list for desired agnles from center; 
        # set attribute values
        self.minus_90_range_index = int(self.center_range_index
                - self.range_elements_in_90_degrees) 
        self.plus_90_range_index  = int(self.center_range_index
                + self.range_elements_in_90_degrees) 
        self.minus_60_range_index = int(self.center_range_index
                - self.range_elements_in_60_degrees)
        self.plus_60_range_index  = int(self.center_range_index
                + self.range_elements_in_60_degrees)
        return False    # set flag so it this code executes only on first /scan msg


    def calculate_cartesian_coords(self, element):
        """ converts polar coordinates to cartesian coords tuple from 
        bearing index and distance ranges[ index ]"""
        self.distance = self.scan_msg.ranges[element]
        self.angle    = self.scan_msg.angle_increment \
                * (element - self.center_range_index)
        self.y        = self.distance * math.sin(self.angle)
        self.x        = self.distance * math.cos(self.angle)
        return (self.x, self.y)


    def calculate_left_error(self):
        """interprets /scan data to identify line of wall and distance from car to left wall"""
        # clear the x & y arrays
        self.left_x_array = []
        self.left_y_array = []
        for element in range( self.plus_60_range_index, 
            self.plus_90_range_index):
            # ignore distances outside lidar range
            if (self.scan_msg.ranges[element] < self.scan_msg.range_max and
                    self.scan_msg.ranges[element] > self.scan_msg.range_min):
                self.result = self.calculate_cartesian_coords(element)
                self.left_x_array.append(self.result[0])
                self.left_y_array.append(self.result[1])

        # fit points to first order polynomial (line)
        if (len(self.left_x_array) > 10):
            self.line_fit_raw = np.polyfit(self.left_x_array, self.left_y_array, 1)
            self.line_fit = np.poly1d(self.line_fit_raw) #  [intercept, slope]
            if self.debug_switch:
                print("slope: %.2d; intercept: %.2d" 
                        % (self.line_fit[0], self.line_fit[1]))
            self.shortest_distance = math.copysign(self.line_fit[0] 
                    / math.sqrt( 1 + self.line_fit[1] ** 2), 1)
            if self.debug_switch:
                    print("shortest distance: %.2f" % self.shortest_distance)
            return [self.set_point - self.shortest_distance, 
                    self.line_fit[1]]    # [shortest distance, slope]
        else:
            return [0, 0]       # not enough data so return zeros


    def calculate_right_error(self):
        """interprets /scan data to identify line of wall and distance from car to right wall"""
        # clear the x & y arrays
        self.right_x_array = []
        self.right_y_array = []
        for element in range(self.minus_60_range_index, 
                self.minus_90_range_index):
            # ignore distances beyond lidar range
            if (self.scan_msg.ranges[element] < self.scan_msg.range_max and
                    self.scan_msg.ranges[element] > self.scan_msg.range_min):
                self.result = self.calculate_cartesian_coords(element)
                self.right_x_array.append(self.result[0])
                self.right_y_array.append(self.result[1])

        # fit points to first order polynomial (line)
        if ( len(self.right_x_array) > 10):
            self.right_line_fit_raw = np.polyfit(self.right_x_array, 
                    self.right_y_array, 
                    1)
            self.right_line_fit = np.poly1d(self.right_line_fit_raw)
            #  [intercept, slope]
            if self.debug_switch:
                print("slope: %.2d; intercept: %.2d"
                         % (self.right_line_fit[0], self.right_line_fit[1]))
            self.shortest_right_distance = math.copysign(self.right_line_fit[0]
                    / math.sqrt(1 + self.right_line_fit[1] ** 2), 1)
            if self.debug_switch:
                print("shortest distance: %.2f" % self.shortest_right_distance)
            return [self.set_point - self.shortest_right_distance, 
                    self.right_line_fit[1]]     # [shortest distance, slope ]
        else:
            return [0, 0]       # not enough data


    def approx_error_derivative(self, error, side):
        """maintains two lists of error distances and times.  (left and right) 
        if list is too short ( <10 elements) returns 0,
        otherwise calculates slope from list of errors and times"""
        if ("left" in side):
            self.left_error_list.append(error)
            t = rospy.Time(self.scan_msg.header.stamp.secs, 
                    self.scan_msg.header.stamp.nsecs)
            self.left_error_times.append(t.to_nsec())

            if self.debug_switch:
                print(self.left_error_list)
                print(self.left_error_times)

            if len(self.left_error_list) > 10:
                self.left_error_list.pop()
                self.left_error_times.pop()
                temp_derivative = np.polyfit(self.left_error_times, 
                    self.left_error_list, 
                    1)
                return_val = (np.poly1d(temp_derivative)[1] * 1000000000)
            else:
                return_val = 0      # not enough data
        else:           #not "left"; must be "right"
            self.right_error_list.append( error)
            t = rospy.Time(self.scan_msg.header.stamp.secs, 
                    self.scan_msg.header.stamp.nsecs)
            self.right_error_times.append(t.to_nsec())

            if self.debug_switch:
                print(self.right_error_list)
                print(self.right_error_times)

            if len(self.right_error_list) > 10:
                self.right_error_list.pop()
                self.right_error_times.pop()
                temp_derivative = np.polyfit(self.right_error_times, 
                        self.right_error_list, 
                        1)
                return_val = (np.poly1d(temp_derivative )[1] * 1000000000)
            else:
                return_val = 0      # not enough data yet
        return return_val


if __name__ == "__main__":
    rospy.init_node( "hst_improved_pd_controller", anonymous = True )
    node = PDControllerNode()
    rospy.spin()
