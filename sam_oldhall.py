#!/usr/bin/env python

#the imports of oldhall include rospy, ackermann messages, and laser messages. their main export is ackermann messages.

import rospy

from ackermann_msgs.msg import AckermannDriveStamped

from sensor_msgs.msg import LaserScan

class DownTheHall():

	def __init__(self): #collects data from laser, passes it to safe
	
		rospy.Subscriber('/scan', LaserScan, self.safe)
		
	def safe(self, data):
	
		msg = AckermannDriveStamped() #creates the command message
	
		dist = data.ranges[135] #this is the distance from 90 degrees to the right of the car.
		
		error = 1 - dist #the error
		
		msg.drive.steering_angle = 0 #steering angle starts at zero
		
		if error > 0: #if the error is positive (it's too close) it steers left.
		
			msg.drive.steering_angle = 0.15
			
		elif error < 0: #if the error is negative (it's too far) it steers right.
			
			msg.drive.steering_angle = -0.15
			
		dist = data.ranges[len(data.ranges) // 2] #finds the distance of whatever's directly in front of the car.
		
		if dist < 2.5: #if something's in front, it steers right. takes priority over all other commands.
		
			msg.drive.steering_angle = 0.3
					
		msg.drive.speed = 1 #speed = 1
		
		#and the next two lines just publish!
				
		pub_command = rospy.Publisher('vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size = 10)
		
		pub_command.publish(msg)
		
if __name__ == '__main__': #CREATES DA FRICKIN NODE AND IT RUNS DA FRICKIN CODE

	rospy.init_node('bang_bang')
	
	node = DownTheHall()
	
	rospy.spin()
