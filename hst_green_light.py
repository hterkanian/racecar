#!/usr/bin/python
import cv2
import numpy as np
from matplotlib import pyplot as plt

# black rectangle (720 x 1280) with 3 circles (green, blue, red)
img = np.zeros( (720, 1280, 3), np.uint8 )
cv2.circle( img, (360, 360), 40, ( 0, 255, 0 ), -1 )
cv2.circle( img, (640, 360), 40, ( 255, 0, 0 ), -1 )
cv2.circle( img, (1000, 360), 40, ( 0, 0, 255 ), -1 )

# ranges for inRange
green_low = np.array( [40, 50, 50 ] )
green_high = np.array( [ 80, 255, 255 ] )

# BGR image to HSV
hsv = cv2.cvtColor( img, cv2.COLOR_BGR2HSV )

# find the green
mask = cv2.inRange( hsv, green_low, green_high )

#blur to smooth the edges
cv2.blur( mask, (10, 10 ) )

res = cv2.bitwise_and(img, img, mask=mask )

img_mod, contours, hierarchy = cv2.findContours( mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE )

cv2.drawContours( img_mod, contours, -1, ( 0, 0, 0 ) )

# how many green regions did we find?
print("number of green regions found: %d" % len(contours))

cv2.imshow('original', img )

cv2.imshow('res', res )

cv2.imshow('img_mod', img_mod )

cv2.waitKey(0)
cv2.destroyAllWindows()
