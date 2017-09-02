#!/usr/bin/python

#LET'S IMPORT LITERALLY FUCKING EVERYTHING

import rospy

from ackermann_msgs.msg import AckermannDriveStamped

from sensor_msgs.msg import LaserScan

#ok, now let's make the class. this is literally the ENTIRE NODE

class IsItSafeNode:
	
	def __init__(self): #when this thing starts, this runs.
	
		self.pub_command = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size = 10) #this says "yo, we're gonna publish to the car."
		
		rospy.Subscriber("/scan", LaserScan, self.safety) #collects the laser scan data, and runs it through self.safety().

		rospy.Timer(rospy.Duration(0.1), rospy.Subscriber("/scan", LaserScan, self.doublecheck)) #every tenth of a second, checks in front of it
		
	def safety(self, data): #self.safety()
	
		minval = min(data.ranges[360:720]) #finds the distance of the closest thing within a 90 degree rane in front of the car

		if minval > 1: #if there's nothing closer than 1 meter
	
			rospy.Subscriber("racecar/ackermann/safety", AckermannDriveStamped, self.publish) #get the commands, and immediately publish them
			
		else: #otherwise,
			
			rospy.Subscriber("racecar/ackermann/safety", AckermannDriveStamped, self.notgood) #take the command and run it through the safety control
			
	def publish(self, msg): #publish the command
	
		self.pub_command.publish(msg)
		
	def notgood(self, msg):

		#if the car isn't going backwards, it sets everything to zero then publishes it.
		
		if msg.drive.speed > 0:
		
			msg.drive.speed = 0
			
		self.pub_command.publish(msg)

	def doublecheck(self, data): #

		if min(data.ranges[360:720]) < 1:
		
			msg = AckermannDriveStamped()
			
			msg.drive.speed = 0
			
			self.pub_command.publish(msg)
		
		else:
			pass
		
if __name__ == '__main__': #creates da node and runs da code!

	rospy.init_node('safety_node', anonymous = True)

	node = IsItSafeNode()

	rospy.spin()
