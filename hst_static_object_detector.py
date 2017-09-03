#!/usr/bin/env python
"""
Title: static_object_detector.py
Author: Ariel Anders and YOUR NAME HERE
Date: July 18, 2017
Short Description: BeaverWorks Summer Institute Awesome Code!


This file contains skeleton code for a static object detector.  Your assignment is to complete the functions specified in this program.

Please refrain from using google to complete this task. Instead,
use the python packages APIs:

    opencv: http://docs.opencv.org/3.0-last-rst/
    numpy: https://docs.scipy.org/doc/numpy/reference/


"""
import cv2
import numpy as np
import sys

class StaticObjectDetector:

    HSV_RANGES = [np.zeros(3,dtype=np.uint8),np.zeros(3,dtype=np.uint8)]

    
    def __init__(self, fname):
        try:
            self.image = cv2.imread(fname)
        except:
            print "filename %s is invalid.  Quitting" % fname
            sys.exit(1)

        hsv = [[1, 255, 172],[ 3, 255, 255]]
        self.setHSV(*hsv)
            
        
    def show(self, image=None, window='image'):
        """ (completed helper function)
        shows an image with cv2.imshow but does not
        use waitKey
        """
        if image is None:
            image = self.image
        cv2.imshow(window, image)

    def startDisplay(self, time=0):
        """ (completed helper function)
        start display after loading image with self.show
        """
        cv2.waitKey(time)
        
    def clear(self, window=None):
        """ (completed helper function)
        clear display or displays
        """
        if window is not None:
            cv2.destroyWindow(window)
        else:        
            cv2.destroyAllWindows()
    

    def setHSV(self, min_hsv, max_hsv):
        """ (completed helper function)
        update minimum and maximum hsv values to use with inrange
        when doing the thresholding
        """
        assert len(min_hsv) == 3 and len(max_hsv) == 3
        min_val = np.array(min_hsv, np.uint8)
        max_val = np.array(max_hsv, np.uint8)
        self.HSV_RANGES = [min_val, max_val]

    def detect(self, debug=True):
        """ (completed helper function)
        object detector function described in pseudo code from lecture 
        Contains debug functions to view seperate components of object detection
        """
        hsv_img = self.convertToHSV(self.image)
        img_thresh = self.thresholdImage(hsv_img)
        contours, labeled_image = self.getContours(img_thresh)
        if len(contours) > 0:
            largest_contour = self.sortContours(contours)[0]
        else:
            largest_contour = None
        if debug:
            print "Largest Contour found", largest_contour
            print "type any key to quit showing windows"

            self.show(self.image,window="original")
            self.show(hsv_img,window="hsv")
            self.show(img_thresh,window="threshold")
            self.show(labeled_image, window="contoursFound")
            self.startDisplay()
            self.clear()
        return largest_contour

    def convertToHSV(self, image):
        return cv2.cvtColor( image, cv2.COLOR_BGR2HSV )

    def thresholdImage(self, image):
        return cv2.inRange(image, self.HSV_RANGES[0], self.HSV_RANGES[1])


    def getContours(self, image):
        img_mod, contours, hierarchy = cv2.findContours( image, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        img = self.image.copy()
        cv2.drawContours(img, contours, -1,  (0, 255, 0), 3)
        return contours, img

    def getKey(self, item):
        return item[1]
    
    def sortContours(self, contours):
        print len(contours)
        contour_list = []
        result = []
        for i in range(len(contours)):
            contour_list.append([contours[i], cv2.contourArea(contours[i])])
        sorted_list = sorted( contour_list, key=self.getKey, reverse=True)
        for i in range(len(sorted_list)):
            result.append(sorted_list[i][1])
        return result

    
    

if __name__=="__main__":
    sod = StaticObjectDetector('cone.png')
    sod.detect()







    


