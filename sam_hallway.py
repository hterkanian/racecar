#!/usr/bin/env python

import rospy

from ackermann_msgs.msg import AckermannDriveStamped

from sensor_msgs.msg import LaserScan

class DownTheHall():

	def __init__(self):
	
		rospy.Subscriber('/scan', LaserScan, self.safe)
		
	def safe(self, data):
	
		tolerance = 1
		
		msg = AckermannDriveStamped()
		
		left = min(data.ranges[:540])
		
		right = min(data.ranges[540:])
		
		turnright = False
		
		turnleft = False
		
		stop = False
		
		if right < tolerance or left < tolerance:
		
			if right < left:
			
				turnright = True
				
			else:
			
				turnleft = True
				
		if min(data.ranges[360:540]) < tolerance and min(data.ranges[540:720]) < tolerance:
		
			stop = True

		if stop:
		
			msg.drive.speed = 0
			
			msg.drive.steering_angle = 0
			
		else:
		
			msg.drive.speed = 1
			
			if turnright:
			
				msg.drive.steering_angle = 20
				
			elif turnleft:
			
				msg.drive.steering_angle = -20
				
			else:
			
				msg.drive.steering_angle = 0
				
		pub_command = rospy.Publisher('vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size = 10)
		
		pub_command.publish(msg)
		
if __name__ == '__main__':

	rospy.init_node('bang_bang')
	
	node = DownTheHall()
	
	rospy.spin()
