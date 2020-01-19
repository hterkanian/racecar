#!/usr/bin/env python
"""
Author: Harry Terkanian
July 25, 2017

Revised June 11, 2019
Revised November 29, 2019; passed publisher objects to callback as args.
Revised January 7, 2020; code and comment cleanup
Revised January 8, 2020; added lines to diagnostic images

Input:      ROS message from Zed Camera with image

Output:     Publishes ROS message with steering angle to steer racecar 
                toward object identified in image (at arbitrary speed 
                set by self.drive_speed);

Diagnostic output (if self.diagnostics = True):
            Publishes processed image to /hst/image; and
            Publishes error value to /hst/error

Debug outputs (if self.debug_switch = True):
            Prints various debugging information to console

Use CvBridge to convert mesage image from Zed camera
Use OpenCV2 to process converted image to locate object(s) in desired 
    color range.

Usage Notes:
    1.  Adjust self.color_range_low & _hi for the desired HSV color range of 
            object(s) being searched for in the image.
    2.  Consider implementing slice of lower portion of image if running on
            racecar.  See #TODO

Requires: ROS Kinetic, Python 2.7, OpenCV 2, cv_bridge
"""


import rospy
import roslib
import math
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped


class HSTObjectFollowerNode:


    def __init__(self):
        #===============Class initialization===================================

        #===============State Variables========================================
        self.color_range_low = np.array([ 90,  50,  50]) 
        self.color_range_hi  = np.array([150, 255, 255])
        self.steering_saturation    = 0.30  # max steering angle (radians)
        self.drive_speed            = 0.50  # arbitrary
        self.Kp                     = 5.0   # proportional steering gain
        self.debug_switch           = False # print debugging statements
        self.publish_diagnostics    = True  # publish diagnostic messages
        self.bridge = CvBridge()        # converts CV2 images to/from  ROS
                
        #===============ROS topic subscriptions & publications=================           
        #===============publisher for Ackermann drive /navigation topic========
        self.pub_cmd = rospy.Publisher(
                "/ackermann_cmd_mux/input/navigation",
                AckermannDriveStamped, queue_size = 5
                )

        #===============publisher for processed image /hst/image topic=========
        self.pub_image = rospy.Publisher("/hst/image", 
                Image, 
                queue_size = 5
                )

        #===============publisher for calculated error /hst/error topic========
        self.pub_error = rospy.Publisher("/hst/error", 
                Float32, 
                queue_size = 5
                )

        #===============subscribe to /zed/left/image_rect_color================
        rospy.Subscriber("/zed/left/image_raw_color",
                Image,
                self.camera_callback,
                [
                    self.pub_cmd,
                    self.pub_image,
                    self.pub_error
                ]
                )
        #===============end ROS subscriptions and publications=================

        if self.debug_switch:
            print("=====Initialization complete====")


    #===================class methods==========================================
    def camera_callback(self, msg, publishers):
        """
        Callback method for camera messages.  
        identifies object(s) in image with color in specified color range; 
        calculates relationship of horizontal location of object center
        to horizontal image center, normalized to range (-1: +1).
        Steering command is issued to turn toward object center.

        sensor_msgs.msg/Image message format:
            msg.header       # ROS message header
            msg.height       # uint32 image height (rows)
            msg.width        # uint32 image width (columns)
            msg.encoding     # string encoding of pixels
                           Color values:
                                RGB8 = "rgb8";
                                RGBA8 = "rgba8";
                                BGR8 = "bgr8";
                                BGRA8 = "bgra8";
                                MONO8="mono8";
                                MONO16="mono16";
            msg.is_bigendian # is data big_endian
            msg.step         # full row length in bytes
            msg.data         # [] matrix data, size is (step * rows)

            Callback process:
                1.  Acquire camera image
                2.  Convert image to OpenCV format
                3.  Select bottom half of image
                4.  Convert to image to HSV color encoding
                5.  Select image pixels in desired color range
                6.  Blur image to eliminate spurious pixels
                7.  Draw contours
                8.  Find largest contour
                9.  Draw bounding rectangle around largest contour
                10. Raw error is horizontal center of image - horizontal 
                        center of bounding rectangle
                11. Normalize the error
                12. Publish the normalized error as a steering correction

            Debugging:
                Set self.debug_switch to True to print diagnostnic information.
                Set self.publish_diagnostics to True to publish two ROS topics:
                    (1) the processed image with marker lines to /hst/image; 
                        and 
                    (2) the error value to /hst/error. 
        """

        # publisher objects
        pub_cmd =   publishers[0]   # AckermannDriveStamped messages
        pub_image = publishers[1]   # Processed image
        pub_error = publishers[2]   # error

        if self.debug_switch:
            print("processing image")

        image_msg = msg

        # extract image and convert to CV2 bgr8 format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, 
                    image_msg.encoding)
        except CvBridgeError as e:
            print('CvBridge error: ', e)    # fatal exception, quit

        if self.debug_switch:
            print("OpenCV image shape: rows: {0}, columns: {1}, color elements: {2}".
                    format(cv_image.shape[0], cv_image.shape[1], cv_image.shape[2]))
            print('Original encoding: ' + image_msg.encoding)

        #Select the bottom half of cv_image
        reduced_img = cv_image[cv_image.shape[0] // 2:, : ,:]

        # Convert image from BGR to HSV format
        if self.debug_switch:
            print('Resized image shape', reduced_img.shape)
            print('Converting image from BGR to HSV.')

        hsv_image =  cv2.cvtColor(reduced_img, cv2.COLOR_BGR2HSV)

        # find any object(s) in image with color in color range
        image_thresh = cv2.inRange(hsv_image, self.color_range_low,
            self.color_range_hi)

        # get rid of spurious pixels by averaging over a block
        cv2.blur(image_thresh, (5, 5))

        if self.debug_switch:
            print("Finding object(s) in image")

        # find the contours
        img_mod, contours, hierarchy = cv2.findContours(
                image_thresh,
                cv2.RETR_CCOMP,
                cv2.CHAIN_APPROX_SIMPLE
                )
        labeled_image = image_thresh.copy()
        cv2.drawContours(labeled_image, contours, -1, (0, 255, 0))

        # identify the index of the largest contour in contours
        if len(contours) > 0:
            largest_contour = 0
            largest_area = cv2.contourArea(contours[0])
            for i in range(len(contours)):
                if cv2.contourArea(contours[i]) > largest_area:
                    largest_contour = i
                    largest_area = cv2.contourArea(contours[i])
        else:
            # no contours found
            largest_contour = None

        if self.debug_switch:
            print('{0} contours found.'.format(len(contours)))
                                       
        if largest_contour == None:
            # no contours found in image; nothing to do
            return
 
        # calculate the error:
        # 1.    find bounding rect; 
        # 2.    calc horizontal center of rect; 
        # 3.    error is image horizontal center - horizontal 
        #       center of rect; and
        # 4.    normalize error to range -1:+1
        rect = cv2.boundingRect(contours[largest_contour])
        rect_center = (rect[0] + rect[2] // 2)
        image_center = labeled_image.shape[1] // 2 
        raw_error = image_center - rect_center
        error = (raw_error  * 1.0) / (image_center * 1.0)

        if self.publish_diagnostics:
            self.pub_err(error, pub_error)
            self.draw_lines(reduced_img, rect)
            self.pub_img(reduced_img, pub_image)

        if self.debug_switch:
            print('Bounding rect: top left coordinates: (x:%i, y:%i), width:%i, height:%i'
                    % (rect[0], rect[1], rect[2], rect[3]))
            print("image center: %i, object center: %i, error: %.2f" 
                    % (image_center, object_center , error))

        # call controller to issue drive command
        self.controller(error, pub_cmd)


    #===================class helper methods===================================
    def draw_lines(self, img, rect):
        """Draw meaningfull lines on image"""
        # blue vertical line at image center
        cv2.line(
                img, 
                (img.shape[1] // 2, 0), 
                (img.shape[1] // 2, img.shape[0]), 
                (255, 0, 0), 
                1
                )
        # green vertical line at rect center
        cv2.line(
                img, 
                (rect[0] + rect[2] // 2, 0), 
                (rect[0] + rect[2] // 2, img.shape[0]), 
                (0, 255, 0), 
                1
                )
        # yellow horizontal arrow for raw error
        cv2.arrowedLine(
                img, 
                (img.shape[1] // 2, rect[1] + rect[3] // 2), 
                (rect[0] + rect[2] // 2, rect[1] + rect[3] // 2), 
                (0, 255, 255), 
                1, 
                tipLength = 0.1
                )
        # red lines around rect box
        cv2.line(
                img, 
                (rect[0], rect[1]), 
                (rect[0], rect[1] + rect[3]), 
                (0, 0, 255), 
                1
                )
        cv2.line(
                img, 
                (rect[0], rect[1]), 
                (rect[0] + rect[2], rect[1]), 
                (0, 0, 255), 
                1
                )
        cv2.line(
                img, 
                (rect[0] + rect[2], rect[1]), 
                (rect[0] + rect[2], rect[1] + rect[3]), 
                (0, 0, 255), 
                1
                )
        cv2.line(
                img, 
                (rect[0], rect[1] + rect[3]), 
                (rect[0] + rect[2], rect[1] + rect[3]), 
                (0, 0, 255), 
                1
                )


    def pub_img(self, labeled_image, pub_image):
        """Publish image to /hst/image_pub"""
        pub_image.publish(self.bridge.cv2_to_imgmsg(labeled_image))


    def pub_err(self, error, pub_error):
        """Publish error to /hst/error_value"""
        err_msg = Float32()
        err_msg.data = error
        pub_error.publish(err_msg)


    def controller(self, error, pub_cmd):
        """
        Applies prorpotional steering gain (self.Kp)
        Caps steering angle at +/- self.steering_saturation
        Issues drive message with arbitrary drive speed and calculated 
        steering angle.
        """
        cmd_msg = AckermannDriveStamped()
        cmd_msg.drive.speed = self.drive_speed

        # apply proportional controller gain (self.Kp)
        raw_steering_angle = error * self.Kp 

        # cap steering angle at self.steering_saturation
        if abs(raw_steering_angle) > self.steering_saturation:
            steering_angle = math.copysign(self.steering_saturation, 
                    raw_steering_angle)
        else:
            steering_angle = raw_steering_angle

        if self.debug_switch:
            print("controller: steering angle: %.2f"
                    % (steering_angle)
                    )

        # insert steering angle and issue drive command
        cmd_msg.drive.steering_angle = steering_angle
        pub_cmd.publish(cmd_msg)


    def display_image(self, title, image):
        cv2.imshow(title, image)
        cv2.waitKey(0)
        cv2.destroyWindow(title)


#=======================execution begins here==================================
if __name__ == "__main__":
    rospy.init_node("hst_object_follower", anonymous = True)
    node = HSTObjectFollowerNode()
    rospy.spin()
