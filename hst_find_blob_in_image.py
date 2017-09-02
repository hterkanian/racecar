#!/usr/bin/python

# Author: Harry Terkanian
# July 25, 2017

import rospy
import roslib
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

class HSTBlobDetectorNode:

        def __init__( self ):

                #========state variables====================
                self.yellow_ranges = [ np.array( [ 20, 150, 10] ), np.array ( [ 70, 255, 255 ] ) ]
                self.debug_switch  = True
                self.error_list    = []

                # for converting ros images to cv2 and back=
                self.bridge = CvBridge()
                
                # ROS topic subscriptions & publications====            
                # subscribe to /zed/rgb/image_rect_color====
                rospy.Subscriber( "/zed/left/image_rect_color", Image, self.camera_callback )

                # publish /hst/image========================
                self.pub_cmd = rospy.Publisher( "/hst/image_pub", Image, queue_size = 10 )

                # publish error=============================
                self.pub_error = rospy.Publisher( "/hst/error_video", Float32, queue_size = 10 )

                if self.debug_switch:
                        print("=====starting====")

        def camera_callback( self, msg ):
                if self.debug_switch:
                        print("processing frame")
                self.msg = msg
                if self.debug_switch:
                        print(self.msg.header)
                        print("height: %d, width: %d" % (self.msg.height, self.msg.width ) )
                        print("encoding: ",  self.msg.encoding )
                        print("array size: %d" % len(self.msg.data) )
                try:
                        # convert to cv2 image
                        cv_image = self.bridge.imgmsg_to_cv2( msg, "bgr8")
                except CvBridgeError as e:
                        print(e)

                # look at bottom quarter of image only to avoid distractions; less to process
                cv_image_reduced = cv_image[ 3 * self.msg.height // 4:, : , : ]

                # BGR => HSV
                hsv_image = self.convertToHSV( cv_image_reduced )

                # find the threshold(s)
                img_thresh = self.thresholdImage( hsv_image )

                # get rid of spurious stuff by averaging over a block
                cv2.blur( img_thresh, ( 5, 75 ) )

                if self.debug_switch:
                        print("finding yellow")

                # find the contours
                contours, labeled_image = self.getContours( img_thresh )

                # id the index of the largest contour
                largest_contour = self.getLargestContour( contours )
                                        
                if self.debug_switch:
                        print(labeled_image.shape)
                        print(labeled_image.dtype)
                        print( "bounding rect contour" )
                        for i in range(len(contours)):
                                print( cv2.boundingRect( contours[i] ) )
                                print( cv2.contourArea( contours[i] ) )
                        print( "largest contour: " )
                        print( contours[largest_contour] )
                        print( "largest contour area: ")
                        print( cv2.contourArea( contours[largest_contour] ) )

                # calculate the error; find bounding rect, calc horizontal center of rect
                # error is distance from center normalized over +0.5 -0.5 range
                rect = cv2.boundingRect( contours[ largest_contour ] )
                line_center = (rect[0] + rect[2] / 2.0 )
                error = ( ( labeled_image.shape[1] / 2 ) - line_center ) / ( labeled_image.shape[1] / 2.0 )
                if self.debug_switch:
                        print( "center point: %.2f, error: %.2f" % (line_center, error)  )
                
                # publish the processed image                
                self.pub_cmd.publish( self.bridge.cv2_to_imgmsg(  labeled_image ) )

                # publish the calculated error
                smoothedError = self.smoothError( error )
                if self.debug_switch:
                        print("smoothedError: %.3f" % smoothedError )
                self.pub_error.publish( smoothedError )
                
        def convertToBGR( self, labeled_image ):
                return cv2.cvtColor( labeled_image, cv2.COLOR_HSV2BGR )

        def convertToHSV( self, bgr_image ):
                if self.debug_switch:
                        print("converting to HSV")
                        print(bgr_image.shape)
                        print(bgr_image.dtype)
                return cv2.cvtColor( bgr_image, cv2.COLOR_BGR2HSV )

        def thresholdImage( self, image ):
                return cv2.inRange( image, self.yellow_ranges[0], self.yellow_ranges[1] )
                
        def getContours(self, image ):
                img_mod, contours, hierarchy = cv2.findContours( image, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE )
                img = image.copy()
                cv2.drawContours( img, contours, -1, ( 0, 255, 0 ) )
                return contours, img

        def getLargestContour( self, contours ):
                if len( contours ) > 0:
                        largest_contour = 0
                        largest_area = cv2.contourArea( contours[0] )
                        for i in range( len( contours ) ):
                                if cv2.contourArea( contours[i] ) > largest_area:
                                        largest_contour = i
                                        largest_area = cv2.contourArea( contours[i] )
                else:
                        largest_contour = None

                return largest_contour

        def smoothError( self, error ):
                """ returns an average of the last 5 errors."""
                self.error_list.append( error )
                if len(self.error_list) > 6:
                        dropped = self.error_list.pop( 0 )         # pop off the oldest
                error_sum = 0
                for i in range(len(self.error_list)):
                        error_sum += self.error_list[i]
                        if self.debug_switch:
                                print("error sum %.2f" % error_sum)
                return error_sum / len(self.error_list)

if __name__ == "__main__":
        rospy.init_node( "hst_blob_detector", anonymous = True )
        node = HSTBlobDetectorNode()
        rospy.spin()



