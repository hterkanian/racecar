#!/usr/bin/python
#

import rospy
import math
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

class SafetyControllerNode:
    def __init__( self ):

	# state variables
	self.timer_duration = rospy.Duration( 0.1 )
	self.safe_distance  = 0.1						# about 10 cm
	self.stop_msg       = False
	self.first_scan_msg = True
	self.begin_scan_arc = 0
	self.end_scan_arc   = 0

	# subscribe to incoming Ackermann drive commands
	rospy.Subscriber( "ackermann_cmd/mux/input/navigation", AckermannDriveStamped, 								self.ackermann_cmd_input_callback )

	# subscribe to lidar scan input
	rospy.Subscriber( "/scan", LaserScan, self.scan_callback )

	# publisher for the safe Ackermann drive command
	self.cmd_pub = rospy.Publisher( "ackermann_cmd/mux/input/safety", AckermannDriveStamped, queue_size = 10 )

	def scan_callback( self, msg ):
		# check for min distance less than safe_distance
		# if found, set stop_msg = True

		if ( first_scan_msg ):
			# first scan msg, calculate arc to scan
			self.first_scan_msg = False		# thru here once only
			self.scan_range_array_length = len( msg.ranges )			# range array length
			self.scan_range_array_center = scan_range_array_length // 2		# range array center
			# calculate number of scans in pi/4 arc
			self.half_arc = ( math.pi / 4.0 ) // msg.data.scan_increment
			self.begin_scan_arc = self.scan_range_center - self.half_arc	# range array to scan
			self.end_scan_arc   = self.scan_range_center + self.half_arc
			rospy.loginfo( "Range length: %f, arc begin: %f, arc end: %f", 							self.scan_range_array_length, self.begin_arc_scan, self.end_arc_scan )
		self.min_val = min( msg.ranges[ self.begin_arc_scan : self.end_arc-scan ] )
		if ( self.min_val < self.safe_distance ):
			self.stop_msg = True

	def ackermann_cmd_input_callback( self, msg ):
		# if no safety issues, republish the input
		if ( stop_msg ):
			# modify msg to reduce speed and angle to zero, e. g. stop
			msg.drive.speed = 0.0
			# republish original message, modified if necssary
		self.cmd_pub.publish( msg )

if __name__ == "__main__":
	rospy.init_node( "safety_controller" )
	node = SafetyControllerNode()
	rospy.spin()


