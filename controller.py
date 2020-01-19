#!/usr/bin/python
"""
    Author Harry Terkanian
    August 27, 2017
    Most recent revision September 28, 2018

    ROS/racecar PD controller

    Currently calculates values for following either wall if selected 
        (see self.follow_left_wall & self.follow_right_wall) but the controller only
        follows left wall.

"""

import rospy
import math
import socket
import re
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan


class WallFollowNode:


    def __init__(self):

        # ==============state class attributes ================================
        self.lidar_offset           = 0.1525    # lidar to side of car (m)
        self.side_of_car_offset     = 0.50      # desired distance: car to wall (m)
        self.steering_saturation    = 0.30	# max steering angle (radians)
        self.drive_speed	    = 0.50	# arbitrary
        self.set_point = self.lidar_offset + self.side_of_car_offset
        self.kp                     = 0.4 / self.steering_saturation # proportional gain
        self.kd			    = 1.0	# arbitrary
        self.first_scan_msg         = True      # flag for self.first_scan()
        self.debug_switch           = True      # Turns on debug print statements
        self.follow_left_wall       = True      # Follow this wall if True
        self.vesc                   = ''	# prepend ackermann msg with /vesc?

        # lidar range data indicies for various angles calculated by first_scan()
        self.range_center_element           = 0	# center of range data
        self.range_elements_in_90_degrees   = 0
        self.range_elements_in_60_degrees   = 0
        self.range_elements_in_30_degrees   = 0
        self.range_elements_in_15_degrees   = 0
        self.range_elements_in_05_degrees   = 0

        # error
        self.left_error_distance            = 0.0
        self.left_error_derivative          = 0.0
        self.left_error_list                = []
        self.left_error_times               = []

        # steering duty cycle
        self.duty_cycle			    = rospy.Duration(1.0)
        self.duty_cycle_load		    = rospy.Duration(0.70)
        self.duty_cycle_start_time	    = rospy.Time.now()

        # ==========Racecar or Gazebo?=========================================
        # ==========Which platform are we running on?==========================
        cpu = re.compile('tegra')
        m = cpu.findall(socket.gethostname())
        if not m:			# no match; not a Jetson so not a racecar
            self.vesc = '/vesc'
        if self.debug_switch:
            print('Ackermann_cmd_mux prepend: ', self.vesc)

        # ==========ROS topic subscriptions & publications=====================
        # ==========subscribe to lidar /scan input=============================
        rospy.Subscriber(
                        "/scan", 
                        LaserScan, 
                        self.scan_callback)
        # ==========publisher for Ackermann drive /navigation topic============
        self.cmd_pub = rospy.Publisher( 
                        self.vesc + "/ackermann_cmd_mux/input/navigation", 
                        AckermannDriveStamped, queue_size = 10)


    # ==================class method definitions ==============================

    def scan_callback(self, msg):
        """Callback method for /scan topic.  Calculates error distance and derivative
        for both left and right sides.
        Calls first_scan() to calculate useful lider index values.
        Checks for obstruction in front of car and turns right if detected
        Calls calculate_left_error() to calc left error & slope.
        Calls approx_error_derivative() to calc error derivative (both sides).
        Calls controller() to issue appropriate instructions to drive to
        maintain desired wall separation.

        Currently  implemented for left wall only."""

        self.scan_msg = msg

        # determine useful indicies into /scan data.ranges[] (once)
        if self.first_scan_msg:
            self.first_scan_msg = self.first_scan()

        # determine error characteristics (error distance and slope)
        if self.follow_left_wall:
            left_error = self.calculate_left_error()
            self.left_error_distance = left_error[0]

            # determine first derivative of left error using time and wall distance
            self.left_error_derivative = self.approx_error_derivative(
                    self.left_error_distance, 
                    "left")

            #issue the AckermannDriveStamped /navigation message
            self.controller()


    # ==================class helper method definitions (functions)=============


    def first_scan(self):
        """ Calculates useful indicies into the /scan range[] and saves values 
        as class attributes.  Called once by scan_callback() on first scan msg."""

        length = len(self.scan_msg.ranges)
        self.range_center_element = length // 2

        # calculate number of scan sweeps in 90, 60, 30, 15 % 5 degree arcs
        self.range_elements_in_90_degrees = int((math.pi / 2.0)\
                // self.scan_msg.angle_increment)
        self.range_elements_in_60_degrees = int((math.pi / 3.0)\
                // self.scan_msg.angle_increment)
        self.range_elements_in_30_degrees = int((math.pi / 6.0)\
                // self.scan_msg.angle_increment)
        self.range_elements_in_15_degrees = int((math.pi / 12.0)\
                // self.scan_msg.angle_increment)
        self.range_elements_in_05_degrees = int((math.pi / 36.0)\
                // self.scan_msg.angle_increment) 

        return False    # set flag so it this code executes only on first /scan msg


    def calculate_left_error(self):
        """ returns the minimum value in the desired portion of the lidar scan range data."""
        min_distance = min(self.scan_msg.ranges[self.range_center_element + self.range_elements_in_30_degrees : \
                self.range_center_element + self.range_elements_in_90_degrees])
        if self.debug_switch:
            print("\n=============\nMin left distance: %.3f" % (min_distance - self.lidar_offset))
        return [(min_distance - self.lidar_offset) - self.set_point, 0]


    def approx_error_derivative(self, error, side):
        """Maintains two lists of error distances and times (left and right). 
        If list is too short ( <20 elements) returns 0,
        otherwise calculates derivative as slope from list of times and errors;
        d(error)/dt; units: meters per second."""
        billion = 1000000000.0
        if ("left" in side):
            self.left_error_list.append(error)
            t = rospy.Time(self.scan_msg.header.stamp.secs, 
                    self.scan_msg.header.stamp.nsecs)
            self.left_error_times.append(t.to_nsec() / billion)

            if len(self.left_error_list) > 10:
                self.left_error_list.pop(0)
                self.left_error_times.pop(0)
                temp_derivative = np.polyfit(self.left_error_times, 
                    self.left_error_list, 
                    1)
                if self.debug_switch:
                    print("derivative: %.2f" % np.poly1d(temp_derivative)[1])
                return_val = (np.poly1d(temp_derivative)[1])    # meters/sec
            else:
                return_val = 0      # not enough data yet, assume all OK
        else:           #not "left"; must be "right"
            print("\n++++++++++\ncalc derivative right:\nWe should not be here!")
        return 0


    def controller(self):
        self.cmd_msg = AckermannDriveStamped()
        self.cmd_msg.drive.speed = self.drive_speed

#===============================================09-27-2018===============
        self.steering_angle = self.kp * self.left_error_distance

        # cap steering angle
        if abs(self.steering_angle) > self.steering_saturation:
            self.steering_angle = math.copysign(self.steering_saturation, self.steering_angle)

        self.steering_angle = self.steering_angle +  self.kd * self.left_error_derivative

        if self.debug_switch:
            print("controller: steering angle: %.2f" 
                    % (self.steering_angle))
        # issue steering command for specified portion of duty cycle (e. g. 10% 0f 1 second)
        if (rospy.Time.now() - self.duty_cycle_start_time) < self.duty_cycle_load:
            self.cmd_msg.drive.steering_angle = self.steering_angle
            self.cmd_pub.publish(self.cmd_msg)
        else:
            self.cmd_msg.drive.steering_angle = 0.0
            self.cmd_pub.publish(self.cmd_msg)

        # if duty cycle has elapsed reset duty_cycle_start_time
        if rospy.Time.now() - self.duty_cycle_start_time > self.duty_cycle:
            self.duty_cycle_start_time = rospy.Time.now()


if __name__ == "__main__":
    rospy.init_node( "wall_follow_pd_controller", anonymous = True )
    node = WallFollowNode()
    rospy.spin()
