#!/usr/bin/python
"""
 Adapted from a piazza post of 7/12/17 by Stephan
 July 26, 2017
 Revised by Harry Sarkis Terkanian September, 2017.
"""
import rospy
import re
from ackermann_msgs.msg import AckermannDriveStamped


class MoveItNode:


    def __init__(self):
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
	"""
	Issue AckermannDriveStamped message.
        Continue publishing move message until either the robot 
	shuts down or the duration timer expires.
	"""
        start_time = rospy.Time.now()
        duration_obj =  rospy.Duration(int(duration))
        while not (rospy.is_shutdown() 
                    or 
                    rospy.Time.now() > duration_obj + start_time
                    ):
            t = rospy.Time.now() -  start_time
            self.msg.header.stamp.secs = float(int(t.to_sec()))
            self.msg.header.stamp.nsecs \
                    = t.to_nsec() \
                    - int(t.to_sec()) * 1000000000
            self.msg.drive.steering_angle = float(steering_angle)
            self.msg.drive.speed = float(speed) 
            self.cmd_pub.publish(self.msg)
            # print(self.msg)
            rate = rospy.Rate(10)   #rate in Hz
            rate.sleep()


    def get_input(self):
	"""
	Prompt for speed or 's' to stop or 'q' to quit.
	Validate input; issue AckermannDriveStamped command if OK.
	"""
        while True:
            raw_speed = raw_input("Enter speed or S to stop or Q to quit: ")
            raw_speed = raw_speed.strip()
            if raw_speed.upper() == "Q":
                self.move(0, 0, 1)
                break
            if raw_speed.upper() == "S":
                raw_speed = 0.0
                input_OK = True
            else:
                speed_match =  self.digit_test.match(raw_speed)
                if not speed_match:
                    print("Speed is: %s, must be numeric" 
                        % raw_speed.strip())
                    input_OK = False

            steering_angle = 0.0
            duration = 1
            if input_OK:
                print("Steering angle: %s, speed: %s, duration: %s"
                        % (steering_angle, raw_speed, duration))
                self.move(raw_speed, steering_angle, duration)


if __name__ == "__main__":
    rospy.init_node("moveItNode", anonymous = True)
    node = MoveItNode()
    node.get_input()


# start moving in a circle (duration in seconds)
# move(speed=1, steering_angle=0.6, duration=5)
# realize you've made a horrible mistake and put the car in reverse
# move(speed=-1, steering_angle=0, duration=3)
