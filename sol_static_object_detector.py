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
            
        hsv = [[1, 255, 172], [3, 255, 255]]
        self.setHSV(*hsv)

        
    def show(self, image=None, window='image'):
        """ (completed helper function)
        shows an image with cv2.imshow but does not
        use waitKey
        """
        if image == None:
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
        largest_contour = self.sortContours(contours)[0]
        if debug:
            print "Largest Contour found", largest_contour
            print "type any key to quit showing windows"
            self.show(hsv_img,window="hsv")
            self.show(img_thresh,window="threshold")
            self.show(labeled_image, window="contoursFound")
            self.startDisplay()
            self.clear()
        return largest_contour

    def convertToHSV(self, image):
        # TODO convert to hsv image
        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        return hsv_img

    def thresholdImage(self, image):
        # TODO use inRange with self.HSV_RANGES
        thresh = cv2.inRange(image, self.HSV_RANGES[0], self.HSV_RANGES[1])
        return thresh

    def getContours(self, image):
        #TODO return contors
        #TODO label image with contours
        #return [], image
        img_mod, contours, hierarchy = cv2.findContours(\
                image,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
        img = self.image.copy()
        cv2.drawContours(img, contours, -1, (0,255,0), 3)
        return  contours, img

    def sortContours(self, contours):
        #TODO sort contours by size
        #return contours
        contour_area = [ (cv2.contourArea(c), (c) ) for c in contours]
        contour_area = sorted(contour_area,reverse=True, key=lambda x: x[0])
        sorted_contours = [c for (area,c) in contour_area]

        return sorted_contours

    
    

if __name__=="__main__":
    sod = StaticObjectDetector('cone.png')
    sod.show()
    sod.detect()







    


