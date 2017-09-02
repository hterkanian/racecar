#!/usr/bin/python

import cv2
import numpy as np
from matplotlib import pyplot as plt
img = cv2.imread('Downloads/heat_map.jpg')
print(img.shape)
half_width  = img.shape[1] // 2
half_height = img.shape[0] // 2
upper_left 	= img[ :half_height , :half_width ]
upper_right = img[ :half_height , half_width: ]
lower_left 	= img[ half_height: , :half_width ]
lower_right = img[ half_height: , half_width: ]

plt.subplot(221), plt.imshow(upper_left), plt.title('upper left')
plt.subplot(222), plt.imshow(upper_right), plt.title('upper right')
plt.subplot(223), plt.imshow(lower_left), plt.title('lower left')
plt.subplot(224), plt.imshow(lower_right), plt.title('lower right')

plt.show()
