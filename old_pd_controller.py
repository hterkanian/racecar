#!/usr/bin/python
# Author Harry Terkanian
# July 19, 2017

import rospy
import math
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class HSTPDControllerNode:


    def __init__( self ):

        # ==============state class attributes (variables)======================
        self.set_point              = 0.50    # about 10 inches
        self.kp                     = 1.00    # proportional gain (arbitrary)
        self.kd                     = 1.00    # derivative gain (aarbitrary)
        self.drive_speed            = 0.5     # arbitrary
        self.steering_saturation    = 0.30    # max steering angle
        self.first_scan_msg         = True    # flag for self.first_scan()
        self.debug_switch           = True    # Turns on debug print statements


        # list ranges for various scan indicies 
        # (calculated by self.do_first_scan()
        self.minus_90_range_index           = 0
        self.plus_90_range_index            = 0
        self.minus_60_range_index           = 0
        self.plus_60_range_index            = 0
        self.scan_range_array_center        = 0
        self.range_elements_in_90_degrees   = 0
        self.range_elements_in_60_degrees   = 0

        # velocity
        self.velocity                       = 0

        # error
        self.error_distance                 = []
        self.error_derivative               = 0
        self.error_list                     = []
        self.error_times        	    = []

        # ==============ROS topic subscriptions & publications=================
        # subscribe to lidar /scan input
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # publish Ackermann drive /navigation topic
        self.cmd_pub = rospy.Publisher( 
                        "/ackermann_cmd_mux/input/navigation", 
                        AckermannDriveStamped, queue_size = 10 )

    # =================class top level methods=================================
    def scan_callback(self, msg):
        """callback method for /scan topic"""
        self.scan_msg = msg

        # determine various indicies into /scan data.ranges[] (once)
        if (self.first_scan_msg):
            self.first_scan_msg = self.first_scan()

        # determine error
        self.error_distance = self.calculate_error_distance()
            #returns [error, slope of wall]
        if self.debug_switch:
            print("error: %.2f, slope: %.2f"
                % (self.error_distance[0], self.error_distance[1]))

        # determine first derivative of error using velocity and wall angle
        self.error_derivative = self.approx_error_derivative(
            self.error_distance[0])
        if self.debug_switch:
            print("error distance: %.2f" % self.error_distance[0])
            print("error derivative: %.2f" % self.error_derivative)

        #issue the AckermannDriveStamped message
        self.controller()

    # ==================class helper method definitions (functions)============ 
    def controller(self):
        self.msg = AckermannDriveStamped()

        self.msg.drive.speed = self.drive_speed

        # calculate steering angle = Kp * e + kd * de/dt
        self.msg.drive.steering_angle = (self.kp * self.error_distance[0] 
            + self.kd * self.error_derivative)
        if self.debug_switch:
            print("error: %.2f, steering gain: %.2f, steering angle: %.2f" 
                % (self.error_distance[0], self.kp, 
                self.msg.drive.steering_angle ))
            print("error derivative: %.2f, kd: %.2d"
                % (self.error_derivative, self.kd))
        # cap steering angle at saturation value
        if (abs(self.msg.drive.steering_angle) > self.steering_saturation):
            self.msg.drive.steering_angle = (self.steering_saturation 
                * math.copysign(1, self.msg.drive.steering_angle))
        if self.debug_switch:
            print('error distance: %.2f, derivative: %.2f steering_angle: %.2f'
                % (self.error_distance[0], 
                self.error_derivative, 
                self.msg.drive.steering_angle))

        self.cmd_pub.publish(self.msg)
        

    def first_scan(self):
        """called once by scan_callback on first scan msg; calculates useful
        indicies into the /scan range[] and sets values of class attributes."""
        self.scan_range_array_length = len(self.scan_msg.ranges)
            # length of range list
        self.scan_range_array_center = self.scan_range_array_length // 2
            # center of range list
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
        return False        # set the flag so we execute this code only once


    def calculate_cartesian_coords(self, element):
        """ converts polar coordinates to cartesian coords tuple for 
        bearing index and distance ranges[index]"""
        self.distance = self.scan_msg.ranges[element]
        self.angle    = self.scan_msg.angle_increment * (element 
            - self.scan_range_array_center)
        self.y        = self.distance * math.sin(self.angle)
        self.x        = self.distance * math.cos(self.angle)
        return (self.x, self.y)
        

    def calculate_error_distance(self):
        """interprets /scan data to identify line of wall 
        and distance from car to left wall"""
        # clear the x & y arrays
        self.x_array = []
        self.y_array = []
        for element in range(self.plus_60_range_index, 
            self.plus_90_range_index ):
            # ignore distances beyond lidar range
            if (self.scan_msg.ranges[element] < self.scan_msg.range_max):
                self.result = self.calculate_cartesian_coords(element)
                self.x_array.append( self.result[0])
                self.y_array.append( self.result[1])

        # fit points to first order polynomial (line)
        if ( len(self.x_array ) > 10):
            self.line_fit_raw = np.polyfit(self.x_array, self.y_array, 1)
            self.line_fit = np.poly1d(self.line_fit_raw)
                # [intercept, slope]
            if self.debug_switch:
                print("slope: %.2d; intercept: %.2d"
                    % (self.line_fit[0], self.line_fit[1]))
            self.shortest_distance = math.copysign(self.line_fit[0]
                / math.sqrt( 1 + self.line_fit[ 1 ] ** 2 ), 1)
            if self.debug_switch:
                print("shortest distance: %.2f" % self.shortest_distance)
            return [self.set_point - self.shortest_distance, self.line_fit[1]]
                # [shortest distance, slope ]
        else:
            return [0, 0]
       

    def approx_error_derivative(self, error):
        """maintains a lsist of error distances and times.  
        if list is too short ( <5) uses
        calculate_error_derivitave to return the derivative 
        else calculate from list of values"""
        self.error_list.append(error)
        t = rospy.Time(self.scan_msg.header.stamp.secs, 
            self.scan_msg.header.stamp.nsecs)
        self.error_times.append(t.to_nsec())
        
        if self.debug_switch:
            print(self.error_list)
            print(self.error_times)

        if len(self.error_list) > 10:
            self.error_list.pop()
            self.error_times.pop()
	    temp_derivative = np.polyfit(self.error_times, self.error_list, 1)
	    return_val = (np.poly1d( temp_derivative )[1] * 1000000000)
        else:
	    return_val = 0
        return return_val
    

if __name__ == "__main__":
    rospy.init_node("hst_pd_controller", anonymous = True)
    node = HSTPDControllerNode()
    rospy.spin()
