#!/usr/bin/python
"""
Author Harry Terkanian

October 3, 2019

Framework for a racecar wall follower (right wall) proportional controller.  
Student code is required to complete this node.  See #TODO and #NOTE comments
below.

Calculate the distance from right wall by evaluating range data in 
LaserScan messages.  Compare the distance to the set_point distance to
calculates the error.

Using the calculated error and proportional constant (kp) issues a steering 
angle.  Insure that the steering angle does not exceed the steering_saturation 
value. 

Publishes the steering angle and a drive speed as an AckermannDriveStamped message.
"""


import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg    import LaserScan


# ============== constants and function definitions ===========================
# ============== constants ====================================================
#TODO: Missing values indicated by "<float>" to be provided by students
lidar_offset            = <float>   # set by student: lidar to side of car (m)
side_of_car_offset      = <float>   # set by student: desired distance: car to wall (m)
drive_speed	    	    = <float>   # arbitrary speed (m/sec); try 1.0
steering_saturation     = 0.30      # max steering angle (radians)
set_point 		  	    = <float>   # set by student; desired distance from wall
                                    # should be lidar_offset + 
                                    # side_of_car_offset
kp                      = <float>   # set by student: proportional gain constant
debug_switch            = True      # Turns on debug print statements if True


# ================== function definitions =====================================


def main():
    """
    Top level function executed if __name__ == '__main__'
    Initializes ROS node, sets up ROS subscriber and publisher, then
    waits for LIDAR messages.
    """


    #==========Initialize ROS node; registers with roscore=====================
    rospy.init_node("WallFollowerController", anonymous=True)


    #==========ROS topic subscription & publisher==============================
    #==========create a publisher object for Ackermann drive /navigation topic=
    drive_pub = rospy.Publisher(
                        "/ackermann_cmd_mux/input/navigation",
                        AckermannDriveStamped,
                        queue_size = 10
                        )


    #==========subscribe to lidar /scan topic==================================
    rospy.Subscriber(
                        "/scan",
                        LaserScan,
                        scan_callback,
                        drive_pub
                        )


    #==========wait for lidar /scan messages===================================
    rospy.spin()


def scan_callback(msg, drive_pub):
        """
	    Called by ROS to process each /scan topic message (LaserScan messages.)

        Arguments are: [1] the /scan message; [2] the navigation publisher object 
        Calculate the error distance from the right wall by calling 
        calculate_right_error() function.

        Issue the appropriate drive system command by calling 
        controller() function to publish an AckermannDriveStamped 
        message to /ackermann_cmd_mux/input/navigation topic.
	    """

        #determine the right side error
        right_error = calculate_right_error(msg)

        #issue the AckermannDriveStamped message
        controller(right_error, drive_pub)


# ==================helper function definitions ===============================


def calculate_right_error(msg):
        """
    	Returns the error based on the LIDAR scan range data and set point.

        Students should determine which portion of the msg.ranges data is 
        pertinent to following the right wall and how to calculate the actual 
        distance.
        """

        #Hint: select portion of the range data by using a Python slice of the 
        #msg.ranges, e. g. msg.ranges[start_index:end_index].
        #TODO: Determine which portion of message range data to look at 
        #TODO: Student code goes here to set wall_distance equal to 
        #distance from the right wall

        error_distance = set_point - wall_distance

        if debug_switch:
            #optional; prints a diagnostic message if debug_switch == True
            print("\n=============\nRight error distance: %.3f" 
                    % (error_distance)
                    )

        return error_distance


def controller(right_error, drive_pub):
        """
        Publishes the desired AckermannDriveStamped drive_msg.
        """

        drive_msg = AckermannDriveStamped()

        #TODO: student code goes here 
        #Calculate the steering angle based on the error and the  proportional
        #controller constant (kp) and assign the result to 
        #drive_msg.steering_angle.
        #NOTE: Calculated steering angle should not exceed the 
        #steering_saturation value (0.30 radians).
        #NOTE: steering angle and steering_saturation values are radians
        #cap steering_angle if necessary and assign to 
        #drive_msg.steering_angle.
        #Assign an arbitrary drive speed value (set in drive_speed constant) 
        #to drive_msg.speed

        if debug_switch:
            #optional; prints a debug message if debug_switch == True
            print("controller: steering angle: %.2f" 
                    % steering_angle
                    )

        drive_pub.publish(drive_msg)
        return 0


if __name__ == "__main__":
    #NOTE: This runs main() only if script is called directly by 
    #the Python interpreter.
    main()
