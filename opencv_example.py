#!/usr/bin/python
"""
Author: Harry Terkanian

July 25, 2017; Most recent revision January 5, 2020

OpenCV demonstration: Finds yellow objects in .jpg image.

Returns offset of object center from image center as a decimal fraction 
of 1/2 image width.

Displays processed image after each transform is applied.
"""


import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt


class OpenCV_example:


    def __init__(self):
        #========state variables=======================
        # OpenCV2 yellow HSV min and max values:
        self.color_ranges = [
                np.array([90, 200, 200]), 
                np.array ([110, 255, 255])
                ]

        # Blur image range (used to smooth image before finding contours):
        self.blur_size = [10, 10]

        # Turn on or off print statements
        self.debug_switch  = True


        #========method definitions===================


    def process(self):
        """
        Main Python function.
        """

        # accquire an image by reading a .jpg file.
        if self.debug_switch:
            print("Acquiring the image by reading the .jpg image file.")
        # NOTE: If running this on the racecar, instead of cv2.imread(), 
        # acquire the image by subscribing to a camera topic and 
        # using ROS CvBridge library to convert each ROS topic message
        # to an OpenCV image.
        # =====acquire the image======
        cv_image = cv2.imread('katie-moun.jpg')

        # print dimensional info about the image and display the image
        if self.debug_switch:
            print('Image shape: ' 
                    + str(cv_image.shape[0]) 
                    + ' by ' 
                    + str(cv_image.shape[1]) 
                    + ' by ' 
                    + str(cv_image.shape[2]) 
                    + ': image size: ' 
                    + str(
                            cv_image.shape[0] 
                            * cv_image.shape[1] 
                            * cv_image.shape[2]
                            )
                    )

        # display the image as read from file
        self.displayImg(cv_image, 'Original Image')

        # convert from native CV2 BGR to RGB color values and re display
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        self.displayImg(rgb_image, 'Image converted from BGR to RGB')

        # look at bottom 512 rows of image only to avoid distractions 
        # and reduce data to process
        cv_image_reduced = rgb_image[rgb_image.shape[0] - 512:, : ,:]
        # print statistics for reduced image and display
        if self.debug_switch:
            print('Sliced image shape: '
                    + str(cv_image_reduced.shape[0])
                    + ' by '
                    + str(cv_image_reduced.shape[1])
                    + ' by '
                    + str(cv_image_reduced.shape[2])
                    + ': sliced image size: '
                    + str(
                            cv_image_reduced.shape[0]
                            * cv_image_reduced.shape[1]
                            * cv_image_reduced.shape[2]
                            )
                    )
        self.displayImg(cv_image_reduced, 'Sliced image, bottom 512 rows')

        # convert from RGB to HSV color values
        if self.debug_switch:
            print('Converting image from BGR to HSV')
            print('BGR image shape: ', cv_image_reduced.shape)
            print('BGR image data type: ', cv_image_reduced.dtype)
        hsv_image = cv2.cvtColor(cv_image_reduced, cv2.COLOR_BGR2HSV)
        if self.debug_switch:
            print('HSV image shape: ', hsv_image.shape)
            print('HSV image data type: ', hsv_image.dtype)

        # find pixels in our color range and display result
        if self.debug_switch:
            print('Finding yellow pixels.')
        img_thresh = cv2.inRange(hsv_image, self.color_ranges[0], self.color_ranges[1])
        self.displayImg(img_thresh, 'Thresholded HSV image')

        # get rid of spurious pixels by averaging over a block of adjacent 
        # pixels and display the result.
        if self.debug_switch:
            print('Blurring image to remove spurious pixels.')
        blured_img = cv2.blur(img_thresh, (self.blur_size[0], self.blur_size[1]))
        self.displayImg(blured_img, 'Blurred thresholded image')

        # draw contours around objects in the image.
        # generates a list of contours and a labeled image
        contours, labeled_image = self.getContours(blured_img)   #helper function, see below
        self.displayImg(labeled_image, 'Labeled image')
        print('There are {0} contours.'.format(len(contours)))
        # print(contours)

        # find the index of the largest contour
        largest_contour = self.getLargestContour(contours)       #helper function, see below
                                        
        if self.debug_switch:
            print('Labeled image shape: ' +  str(labeled_image.shape))
            print('Labeled image dtype: ' + str(labeled_image.dtype))
            print('Index of largest contour: '+ str(largest_contour))
            print('Area of largest contour: ' + str(cv2.contourArea(contours[largest_contour])))
            print('Calculating error')

        # calculate the error: 
        # 1. find the bounding rectangle enclosing the object 
        # with the largest area. 
        # 2. calculate horizontal center of bounding rectangle.
        # 3. calculate horizontal center of image.
        # error is distance from center of the bounding rectangle
        # to the center of the image and normalize over -1 to +1 range
        # NOTE: If running on the racecar the error would be used as input to a steering controller.
        rect = cv2.boundingRect(contours[largest_contour])
        if self.debug_switch:
            print('Bounding rectangle of the largest contour: ', rect)
        object_center = (rect[0] + rect[2] / 2.0)
        image_center  = labeled_image.shape[1] / 2
        error = (image_center - object_center) / (image_center)
        if self.debug_switch:
            print("Calculated points. Image center: %.2f, Detected object center: %.2f, error: %.2f" 
                    % (image_center, object_center, error)
                    )


    def displayImg(self, img, title):
        """
        Utility function to display image with title.
        """
        plt.imshow(img)
        plt.title(title)
        plt.show()


    def getContours(self, image):
        """
        Draw contours around groups of contiguous pixels in color range.
        """
        _, contours, hierarchy = cv2.findContours(image, 
            mode=cv2.RETR_CCOMP, 
            method=cv2.CHAIN_APPROX_SIMPLE)
        img = image.copy()
        cv2.drawContours(img, contours, -1, (0, 255, 0))
        return contours, img


    def getLargestContour(self, contours):
        """
        Find contour with largest area.  Returns index of contour or None.
        """
        if len(contours) > 0:
            largest_contour = 0
            largest_area = cv2.contourArea(contours[0])
            for i in range(len(contours)):
                if cv2.contourArea(contours[i]) > largest_area:
                    largest_contour = i
                    largest_area = cv2.contourArea(contours[i])
        else:
            largest_contour = None
        return largest_contour


#========start here=========================
if __name__ == "__main__":
    obj = OpenCV_example()
    obj.process()
