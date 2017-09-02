# adapted from a piazza post of 7/12/17 by Stephan
# July 26, 2017

import rospy
from ackermann_msgs.msg import AckermannDriveStamped

class MoveItNode:
	def __init__( self ):
	    # initialize the message using the arguments provided
	    msg = AckermannDriveStamped()
	    msg.drive.speed = speed
	    msg.drive.steering_angle = steering_angle
	    # pretty sure this is what the frame_id should be?
	    # probably doesn't really matter, don't think it's used for anything
	    msg.header.frame_id = '/base_link'
    

	def move(speed, steering_angle, duration):   
	    # continue publishing our move message until either the robot shuts down or the timer runs out
    	start_time = time()
    	while not rospy.is_shutdown() and start_time + duration > time():
        	msg.header.stamp = rospy.Time.now()
        	pub.publish(msg)
        	# makes the loop run only 10 times a second (configurable)
        	r.sleep()
    
		    loginfo('Done move (speed %s, steering_angle %s, duration %s)', speed, steering_angle, duration)


if __name__ == "__main__":
	rospy.init_node( "hst_steering_controller", anonymous = True )
	node = MoveItNode()

# add code to accept input from command line or quit

# start moving in a circle (duration in seconds)
# move(speed=1, steering_angle=0.6, duration=5)
# realize you've made a horrible mistake and put the car in reverse
# move(speed=-1, steering_angle=0, duration=3)
