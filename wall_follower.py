#!/usr/bin/python
"""
Author Harry Terkanian

August 27, 2017
Most recent revision February 26, 2019

July 19, 2019: /ackermann_cmd_mux/input/navigation prepended with '/vesc'
    for use in Gazebo racecar-tunnel

Calculates values for following left or right wall.
(left wall selected by self.follow_left_wall = True;
right wall selected by  = False) 
	
Checks for obstructions immediately in front of car 
and issues a hard left (if following right wall) 
or right (if following left wall) turn if detected.
"""

import rospy
import math
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan


class WallFollowerNode:


    def __init__(self):

        # ==============state class attributes ================================

        self.lidar_offset           = 0.1525    # lidar to side of car (m)
        self.side_of_car_offset     = 0.50      # desired distance: car to wall (m)
        self.steering_saturation    = 0.30	    # max steering angle (radians)
        self.drive_speed	    	= 2.0	    # arbitrary
        self.set_point 		  	    = self.lidar_offset + self.side_of_car_offset
        self.kp                     = 0.4 / self.steering_saturation # proportional gain
        self.kd			    	    = 1.0	    # arbitrary
        self.first_scan_msg         = True      # flag for self.first_scan()
        self.debug_switch           = True      # Turns on debug print statements
        self.follow_left_wall       = True      # Follow this wall if True

        # lidar range data indicies for various angles calculated by first_scan()
        self.range_center           = 0	        # center of range data
        self.range_90_degrees       = 0
        self.range_60_degrees       = 0
        self.range_30_degrees       = 0
        self.range_15_degrees       = 0
        self.range_05_degrees       = 0
        self.range_01_degrees       = 0
        self.sin_theta	            = 0.0       # sin(4 * radians per scan)

        # error & error derivative
        self.left_error_distance    = 0.0
        self.left_error_derivative  = 0.0
        self.left_error_list        = []
        self.left_error_times       = []
        self.right_error_distance   = 0.0
        self.right_error_derivative = 0.0
        self.right_error_list       = []
        self.right_error_times      = []


        # ==========ROS topic subscriptions & publications=====================

        # ==========subscribe to lidar /scan input=============================
        rospy.Subscriber(
                        "/scan", 
                        LaserScan, 
                        self.scan_callback
                        )

        # ==========publisher for Ackermann drive /navigation topic============
        self.cmd_pub = rospy.Publisher( 
                        "/vesc/ackermann_cmd_mux/input/navigation", 
                        AckermannDriveStamped, queue_size = 10
                        )


    # ==================class method definitions ==============================

    def scan_callback(self, msg):
        """
	    Callback method for /scan topic. 
        Discards scans with incorrect number of range elements !=1081 
        Calculates error distance and derivative for left or right wall.
        Calls first_scan() to calculate useful lidar index values.
        Checks for obstruction in front of car and turns left or right 
        if detected.
        Calls calculate_left_error() to calc left error.
        Calls calculate_right_error() to calc right error.
        Calls approx_error_derivative() to calc error derivative (both sides).
        Calls controller() to issue appropriate instructions to drive to
        maintain desired wall separation.
	    """

        self.scan_msg = msg

        # discard scan messages that are truncated or are too long
        if len(self.scan_msg.ranges) != 1081:
            return

        # calculate useful indicies into /scan_msg.ranges[] (once)
        if self.first_scan_msg:
            self.first_scan_msg = self.first_scan()

        # check for wall directly in front of car
        if min(self.scan_msg.ranges[self.range_center - 4 : self.range_center + 4]) < 1.75:
            turning = self.detect_turn()
            if turning:
                return

        if self.follow_left_wall:
            # determine left side error
            self.left_error_distance = self.calculate_left_error()

            # determine first derivative of left error
            self.left_error_derivative = self.approx_error_derivative(
                    self.left_error_distance 
                    )
        else:
            # following right wall; determine right side error
    
            self.right_error_distance = self.calculate_right_error()

            # determine first derivative of right error
            self.right_error_derivative = self.approx_error_derivative(
                    self.right_error_distance 
                    )

        # issue the AckermannDriveStamped ... /input/navigation message
        self.controller()


    # ==================class helper method definitions (functions)=============


    def first_scan(self):
        """ 
	    Calculates useful indicies into the /scan range[] and saves values 
        as class attributes.  Called once by scan_callback() on first scan msg.
    	"""

        length = len(self.scan_msg.ranges)
        self.range_center = length // 2

        # calculate number of scan sweeps in 90, 60, 30, 15, 5 & 1 degree arcs
        self.range_90_degrees = int((math.pi / 2.0)\
                // self.scan_msg.angle_increment)
        self.range_60_degrees = int((math.pi / 3.0)\
                // self.scan_msg.angle_increment)
        self.range_30_degrees = int((math.pi / 6.0)\
                // self.scan_msg.angle_increment)
        self.range_15_degrees = int((math.pi / 12.0)\
                // self.scan_msg.angle_increment)
        self.range_05_degrees = int((math.pi / 36.0)\
                // self.scan_msg.angle_increment)
        self.range_01_degrees = int((math.pi / 180.0)\
                // self.scan_msg.angle_increment) 
        self.sin_theta = math.sin(self.scan_msg.angle_increment * 4)

        return False    # set flag so this code executes only on first /scan msg


    def detect_turn(self):
        """
        Calculate approx area of scan for each side of center.  
        If following left wall and right > left area, turn right.
        If following right wall and left > right area, turn left.
        """

        left_list  = self.scan_msg.ranges[self.range_center \
                : self.range_center + self.range_90_degrees : self.range_01_degrees]
        right_list = self.scan_msg.ranges[self.range_center - self.range_90_degrees \
                : self.range_center : self.range_01_degrees] 
        left_area  = 0.0
        right_area = 0.0

        # calculate area on both sides
        for index in range(len(left_list) - 1):
            left_area = left_area \
                    + left_list[index] \
                    * left_list[index + 1] \
                    * self.sin_theta
            right_area = right_area \
                    + right_list[index] \
                    * right_list[index + 1] \
                    * self.sin_theta

        if self.debug_switch:
            print("left area: " 
                    + str(left_area) 
                    + " right area: " 
                    + str(right_area)
                    )

        # decide whether to turn
        if (right_area > left_area) and self.follow_left_wall:
            if self.debug_switch:
                print("++++++++++++++turning hard right\n")
            self.hard_right_turn()
            return_val = True
        elif (left_area > right_area) and not self.follow_left_wall:
            if self.debug_switch:
                print("++++++++++++++turning hard left\n")
            self.hard_left_turn()
            return_val = True
        else:
            return_val = False

        return return_val       # True if we turned; otherwise False


    def calculate_left_error(self):
        """
    	Returns the minimum value in the desired portion of the 
        lidar scan range data (30 to 90 degrees left of center.)
	    """

        min_distance = min(self.scan_msg.ranges[self.range_center + self.range_30_degrees : \
                self.range_center + self.range_90_degrees])
        error_distance = min_distance - self.lidar_offset - self.set_point
        if self.debug_switch:
            print("\n=============\nLeft error distance: %.3f" 
                    % (error_distance)
                    )
        return error_distance


    def calculate_right_error(self):
        """
        Returns the minimum value in the desired portion of the 
        lidar scan range data (30 to 90 degrees right of center.)
        """
        min_distance = min(self.scan_msg.ranges[self.range_center - self.range_90_degrees : \
                self.range_center - self.range_30_degrees])
        error_distance = min_distance - self.lidar_offset - self.set_point
        if self.debug_switch:
            print("\n=============\nRight error distance: %.3f" 
                    % (error_distance)
                    )
        return error_distance


    def approx_error_derivative(self, error):
        """
	    Maintains two lists of error distances and times (left and right). 
        If list is too short (<10 elements) returns 0,
        otherwise calculates derivative as slope from list of times and errors;
        d(error)/dt; units: meters per second.
	    """

        temp_derivative = 0.0
        derivative = 0.0
        billion = 1000000000.0
        if self.follow_left_wall:
            # derivative of left side error
            self.left_error_list.append(error)
            t = rospy.Time(self.scan_msg.header.stamp.secs, 
                    self.scan_msg.header.stamp.nsecs)
            self.left_error_times.append(t.to_nsec() / billion)

            if len(self.left_error_list) > 10:
                self.left_error_list.pop(0)
                self.left_error_times.pop(0)
                temp_derivative = np.polyfit(self.left_error_times, 
                        self.left_error_list, 
                        1
                        )
                derivative = np.poly1d(temp_derivative)[1]
                if self.debug_switch:
                    print("Left error derivative: %.2f" % derivative)
                return_val = derivative    # meters/sec
            else:
                return_val = 0      # insufficient number of samples, return 0
        else:
            # derivataive of right side error
            self.right_error_list.append(error)
            t = rospy.Time(self.scan_msg.header.stamp.secs,
                    self.scan_msg.header.stamp.nsecs)
            self.right_error_times.append(t.to_nsec() / billion)

            if len(self.right_error_list) > 10:
                self.right_error_list.pop(0)
                self.right_error_times.pop(0)
                temp_derivataive = np.polyfit(self.right_error_times,
                        self.right_error_list,
                        1
                        )
                derivative = np.poly1d(temp_derivative)[1]
                if self.debug_switch:
                    print("Right error derivative: %.2f" % derivative)
                return_val = derivative    # meters/sec
            else:
                return_val = 0      # insufficient number of samples, return 0
        return return_val


    def hard_right_turn(self):
        """
        Called if following left wall and right corner detected ahead; 
        slows to half speed and issues hard right turn.
        """

        right_msg = AckermannDriveStamped()
        right_msg.drive.speed = self.drive_speed / 2.0
        right_msg.drive.steering_angle = -0.30

        if self.debug_switch:
            print("Hard right turn.  Steering angle: %.2f" % right_msg.drive.steering_angle)
        self.cmd_pub.publish(right_msg)


    def hard_left_turn(self):
        """
        Called if following right wall and left corner detected ahead;
        slows to half speed and issues hard left turn
        """

        left_msg = AckermannDriveStamped()
        left_msg.drive.speed = self.drive_speed / 2.0
        left_msg.drive.steering_angle = 0.30

        if self.debug_switch:
            print("Hard left turn.  Steering angle: %.2f" % left_msg.drive.steering_angle)
        self.cmd_pub.publish(left_msg)


    def controller(self):
        """
        Caps steering angle if required and issues desired 
        AckermannDriveStamped cmd_msg.
        """

        cmd_msg = AckermannDriveStamped()
        cmd_msg.drive.speed = self.drive_speed

        # add proportional term
        if self.follow_left_wall:
            steering_angle = self.kp * self.left_error_distance
        else:
            steering_angle = self.kp * self.right_error_distance * -1.0

        # add derivative term
        if self.follow_left_wall:
            steering_angle = steering_angle + (self.kd * self.left_error_derivative)
        else:
            steering_angle = steering_angle + (self.kd * self.right_error_derivative * -1.0)

        # cap steering angle at self.steering_saturation radians
        if abs(steering_angle) > self.steering_saturation:
            steering_angle = math.copysign(self.steering_saturation, steering_angle)

        cmd_msg.drive.steering_angle = steering_angle

        if self.debug_switch:
            print("controller: steering angle: %.2f" 
                    % steering_angle
                    )

        self.cmd_pub.publish(cmd_msg)
        return 0


if __name__ == "__main__":
    rospy.init_node("WallFollowerController")
    node = WallFollowerNode()
    rospy.spin()
