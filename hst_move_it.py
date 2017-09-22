# adapted from a piazza post of 7/12/17 by Stephan
# July 26, 2017
# revised by Harry Sarkis Terkanian September, 2017

import rospy
import re
from ackermann_msgs.msg import AckermannDriveStamped


class MoveItNode:


    def __init__( self ):
        self.digit_test = re.compile('-?\d?[.]?\d$|-?\d*[.]\d?')
        # initialize the message using the arguments provided
        self.msg = AckermannDriveStamped()
        self.msg.header.frame_id = '/base_link'
        self.cmd_pub = rospy.Publisher(
                    "/vesc/ackermann_cmd_mux/input/navigation",
                    AckermannDriveStamped,
                    queue_size = 10
                    )


    def move(self, speed, steering_angle, duration):
        # continue publishing our move message until
        # either the robot shuts down or the timer runs out
        start_time = rospy.Time.now()
        duration_obj =  rospy.Duration(int(duration))
        while not (rospy.is_shutdown() 
                    or 
                    rospy.Time.now() > duration_obj + start_time
                    ):
            t = rospy.Time.now() -  start_time
            self.msg.header.stamp.secs = t.to_sec()
            self.msg.header.stamp.nsecs = t.to_nsec()
            self.msg.drive.steering_angle = float(steering_angle)
            self.msg.drive.speed = float(speed) 
            self.cmd_pub.publish(self.msg)
            print(self.msg)
            rate = rospy.Rate(10)   #rate in Hz
            rate.sleep()


    def get_input(self):
        while True:
            raw_steering_angle = raw_input(
                        "Enter steering angle or Q to quit: ")
            raw_steering_angle = raw_steering_angle.strip()
            if raw_steering_angle.upper() == "Q":
                break
            raw_speed = raw_input("Enter speed: ")
            raw_duration = raw_input("Enter duration: ")
            steering_angle_match = self.digit_test.match(raw_steering_angle)
            input_OK = True
            if not steering_angle_match:
                print( "steering angle is: %s must be numeric" 
                        % raw_steering_angle)
                input_OK = False
            else:
                steering_angle = steering_angle_match.group()
            speed_match =  self.digit_test.match(raw_speed)
            if not speed_match:
                print("Speed is: %s, must be numeric" 
                        % raw_speed.strip())
                input_OK = False
            else:
                speed = speed_match.group()
            duration_match = self.digit_test.match(raw_duration)
            if not duration_match:
                print("Duration is: %s, must be numeric" 
                        % raw_duration.strip())
                input_OK = False
            else:
                duration = duration_match.group()
            if input_OK:
                print("Steering angle: %s, speed: %s, duration: %s"
                        % (steering_angle, speed, duration))
                self.move(speed, steering_angle, duration)


if __name__ == "__main__":
    rospy.init_node("moveItNode", anonymous = True)
    node = MoveItNode()
    node.get_input()


# start moving in a circle (duration in seconds)
# move(speed=1, steering_angle=0.6, duration=5)
# realize you've made a horrible mistake and put the car in reverse
# move(speed=-1, steering_angle=0, duration=3)
