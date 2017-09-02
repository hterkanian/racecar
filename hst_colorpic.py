#!/usr/bin/python

import cv2
import numpy as np
from matplotlib import pyplot as plt

im = cv2.imread('Downloads/heat_map.jpg')
hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
for i in range(3):
	h = hsv[:, :, i]
	plt.imshow(h)
	plt.show()
for i in range(3):
	plt.imshow(im[:, :, i])
	plt.show()


