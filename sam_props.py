#!/usr/bin/env python

import rospy

from ackermann_msgs.msg import AckermannDriveStamped

from sensor_msgs.msg import LaserScan

class DownTheHall():

	def __init__(self):
	
		rospy.Subscriber('/scan', LaserScan, self.safe)
		
	def safe(self, data):
	
		msg = AckermannDriveStamped()
	
		dist = min(data.ranges[:375])
		
		error = 1 - dist
		
		msg.drive.steering_angle = error / 2.25
					
		msg.drive.speed = 1.5
				
		pub_command = rospy.Publisher('vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size = 10)
		
		pub_command.publish(msg)
		
if __name__ == '__main__':

	rospy.init_node('bang_bang')
	
	node = DownTheHall()
	
	rospy.spin()
