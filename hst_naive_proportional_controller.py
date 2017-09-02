#!/usr/bin/python
# Author Harry Terkanian
# July 19, 2017

import rospy
import math
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

class NaieveProportionalControllerNode:
	def __init__( self ):

		# state variables
		self.set_point 						= 0.25				# about 10 inches
		# self.correction_angle				= 0					# about 22.5 degrees
		self.k_p							= -0.2				# proportional gain
		self.drive_speed					= 1
		self.first_scan_msg 				= True
		# list ranges for various scan indexes
		self.minus_90_range_index			= 0
		self.plus_90_range_index			= 0
		self.minus_60_range_index			= 0
		self.plus_60_range_index			= 0
		self.scan_range_array_center		= 0
		self.range_elements_in_90_degrees	= 0
		self.range_elements_in_60_degrees	= 0

		self.x_array 						= []
		self.y_array 						= []

		self.error_distance		 			= 0

		# subscribe to lidar scan input
		rospy.Subscriber( "/scan", LaserScan, self.scan_callback )

		# publisher for Ackermann drive navigation topic
		self.cmd_pub = rospy.Publisher( 
						"/vesc/ackermann_cmd_mux/input/navigation", 
						AckermannDriveStamped, queue_size = 10 )

	def scan_callback( self, msg ):
		# determine various indicies into /scan data.ranges[] (once)
		# by calling self.first-scan()
		#
		# interprets /scan data to identify line of left wall and distance from car to wall
		#
		# issue steering command based on fixed correction angle and sign of error

		self.scan_msg = msg

		if ( self.first_scan_msg ):
			self.first_scan_msg = self.first_scan()

		self.error_distance = self.calculate_error_distance()
		# print( self.error_distance )

		self.msg = AckermannDriveStamped()
		self.msg.drive.speed = self.drive_speed * self.k_p * self.error_distance
		# self.msg.drive.steering_angle = math.copysign( self.correction_angle, self.error_distance )
		self.msg.drive.steering_angle = self.k_p * self.error_distance
		if math.copysign( self.msg.drive.steering_angle, 1 ) > .30 :
			self.msg.drive.steering_angle = 0.30 * math.copysign( 1, self.msg.drive.steering_angle )
		print('error distance: %.2f, speed: %.2f steering_angle: %.2f' % 
						( self.error_distance, self.msg.drive.speed, self.msg.drive.steering_angle ) )
		self.cmd_pub.publish( self.msg )

	def first_scan( self ):
		# called once on first scan msg, calculate portion of range to examine
		self.scan_range_array_length = len( self.scan_msg.ranges )			# range array length
		self.scan_range_array_center = self.scan_range_array_length // 2 	# range array center
		# calculate number of scans in pi/4 arc
		self.range_elements_in_90_degrees = ( math.pi / 2.0 ) // self.scan_msg.angle_increment
		self.range_elements_in_60_degrees = ( math.pi / 3.0 ) // self.scan_msg.angle_increment
		# determine beginning and end of range array to scan for needed angles
		self.minus_90_range_index = int( self.scan_range_array_center - self.range_elements_in_90_degrees )	
		self.plus_90_range_index  = int( self.scan_range_array_center + self.range_elements_in_90_degrees )	
		self.minus_60_range_index = int( self.scan_range_array_center - self.range_elements_in_60_degrees )
		self.plus_60_range_index  = int( self.scan_range_array_center + self.range_elements_in_60_degrees )
		print( 'ranges index for -90 degrees: %d' % self.minus_90_range_index )
		print( 'ranges index for +90 degrees: %d' % self.plus_90_range_index )
		print( 'ranges index for -60 degrees: %d' % self.minus_60_range_index )
		print( 'ranges index for +60 degrees: %d' % self.plus_60_range_index )
		return False		# set the flag so we don't come thru here again

	def calculate_cartesian_coords( self, element ):
		# returns (x, y) coords for given ranges[ index ] element
		self.distance = self.scan_msg.ranges[ element ]
		self.angle    = self.scan_msg.angle_increment * ( element - self.scan_range_array_center )
		self.y		  = self.distance * math.sin( self.angle )
		self.x		  = self.distance * math.cos( self.angle )
		return ( self.x, self.y )
		

	def calculate_error_distance( self ):
		for element in range( self.plus_60_range_index, self.plus_90_range_index ):
			if ( self.scan_msg.ranges[ element ] < self.scan_msg.range_max ):		# ignore distances beyond lidar range
				self.result = self.calculate_cartesian_coords( element )
				self.x_array.append( self.result[ 0 ] )
				self.y_array.append( self.result[ 1 ] )

		self.line_fit_raw = np.polyfit( self.x_array, self.y_array, 1 )
		self.line_fit = np.poly1d( self.line_fit_raw )		# (intercept, slope )
		# print( self.line_fit )
		# print( "slope: %.2d; intercept: %.2d" % ( self.line_fit[ 0 ], self.line_fit[ 1 ] ) )
		self.shortest_distance = self.line_fit[0] / math.sqrt( 1 + self.line_fit[ 1 ] ** 2 )
		# print( "shortest distance: %.2f" % self.shortest_distance )
		return self.set_point - self.shortest_distance

if __name__ == "__main__":
	rospy.init_node( "hst_naive_proportional_controller", anonymous = True )
	node = NaieveProportionalControllerNode()
	rospy.spin()


