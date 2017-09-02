#!/usr/bin/env python

#the line above makes it a python script, the lines below make it a ros script

import rospy

from ackermann_msgs.msg import AckermannDriveStamped

from sensor_msgs.msg import LaserScan

class DownTheHall(): #the class contains everything

        def __init__(self): #this function just runs the callback function

                rospy.Subscriber('/scan', LaserScan, self.safe)

        def safe(self, data):

                msg = AckermannDriveStamped() #the message is created - msg is the drive commando

                center = len(data.ranges) // 2 #finds the center

		#the following block creates the minimum distances within a certain range on either side

                rightmin = center / 4

                rightmax  = rightmin*3

                leftmin = len(data.ranges) - rightmin

		leftmax = len(data.ranges) - rightmax

                left = min(data.ranges[leftmax:leftmin])

                right = min(data.ranges[rightmin:rightmax])

		#if there are no values closer than maxo, it sets it to maxo to prevent too much turning. maxo is basically how far it can detect

		maxo = 3

                if left > maxo:

                        left = maxo

                if right > maxo:

                        right = maxo

                setp = (left + right) / 2 #setp is the distance it wants to be from both walls

                dist = min(right, left) #dist is the closest wall

                error = setp - dist #this is how close it is to the setpoint - the ERROR!

                msg.drive.steering_angle = error / 2.25 #calculates steering angle - 2.25 is the gain and it's arbitrary but it works
                
                if right > left: #makes it so that it can turn both ways
                
                	msg.drive.steering_angle = 0 - msg.drive.steering_angle
                
                try: #this is in a try except loop for the first iteration, so that there's no NameError because last is undefined.
                
                	derivative = last - error #this finds the derivative, and subtracts a smaller version of it from the angle. the gain can be anywhere in the range of 0.1 to 0.2
                	#but i put it low for safety.
                	
                	msg.drive.steering_angle -= derivative * 0.1
                
                except(NameError):
                
                	pass

                front = data.ranges[center] #lydar value in front

                msg.drive.speed = 0.75 #sets speed

		#slows down proportionally to a stop. doesn't back up or avoid yet.

                if front < 1:

                        msg.drive.speed = front - 0.5

			if msg.drive.speed < 0:

                                msg.drive.speed = 0

		#aaaaand puuubbblish! /vesc/ackermann_cmd_mux/input/navigation is the topic for gazebo, /ackermann_cmd_mux/input/teleop is for the car itself

                pub_command = rospy.Publisher('/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size = 10)

                pub_command.publish(msg)

		last = error #sets last

if __name__ == '__main__': #creates da node and runs da code

        rospy.init_node('derision')

        node = DownTheHall()

        rospy.spin()
