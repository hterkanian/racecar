#!/usr/bin/env python

#import/ setup

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

#main control function

def control():
	#import laser data
	#if ob. in both
	#if ob in left (turn right)
	#if ob in right (turn left)

#slow down/ stop function

def slow_down():

#turn left function

def left_lobe():
	#turn porpotionally to closeness of min input using a constant derived from the lazer #
	#closeness to 540 determines constant
	#constant is applied to distance to get angle of turn
	min_left = min(data.ranges[540,1081])
	#ang = (540 + index)

#turn right function

def right_lobe():
	min_right = min(data.ranges[0,541])
	#ang = (540 - index)
