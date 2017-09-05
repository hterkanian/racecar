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
        # continue publishing our move message until
        # either the robot shuts down or the timer runs out
        start_time = time()
        while not rospy.is_shutdown() and start_time + duration > time():
            msg.header.stamp = rospy.Time.now()
            pub.publish(msg)
            # makes the loop run only 10 times a second (configurable)
            r = rospy.Rate(10)

            loginfo('Done move (speed %s, steering_angle %s, duration %s)',
                speed,
                steering_angle,
                duration)


if __name__ == "__main__":
    rospy.init_node("moveItNode", anonymous = True)
    node = MoveItNode()

    while True:
        steering_angle = raw_input("Enter steering angle or Q to quit")
        speed = raw_input("Enter speed")
        duration = raw_input("Enter duration")
        if steering_angle == "Q":
            break
        else:
            if not steering_angle.isnumeric():
                print( "steering angle is: %.2d must be numeric" % steering_angle)
            elif not speed.isnumeric():
                print("Speed is: %.2d, must be numeric" % speed)
            elif not  duration.isnumeric():
                print("Duration is: %.2d, must be numeric" % duration)
            else:
                move(speed, steering_angle, duration)


# start moving in a circle (duration in seconds)
# move(speed=1, steering_angle=0.6, duration=5)
# realize you've made a horrible mistake and put the car in reverse
# move(speed=-1, steering_angle=0, duration=3)
