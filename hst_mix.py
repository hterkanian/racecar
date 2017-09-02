#!/usr/bin/python

import cv2
import numpy as np
from matplotlib import pyplot as plt

img = cv2.imread('Downloads/heat_map.jpg')
print(img.shape)
half_width  = img.shape[1] // 2
half_height = img.shape[0] // 2
print( half_height, half_width )

upper_left 	= img[ :half_height , :half_width ]
upper_right = img[ :half_height , half_width+1: ]
lower_left 	= img[ half_height: , :half_width ]
lower_right = img[ half_height: , half_width+1: ]

img2 = np.zeros((img.shape[0], img.shape[1], img.shape[2]), img.dtype )

img2[ :half_height, :half_width   ]   = lower_right	# into upper left
img2[ half_height:, half_width+1: ]   = upper_left	# into lower right
img2[ half_height:, :half_width   ]   = upper_right	# into lower left
img2[ :half_height, half_width+1: ]   = lower_left	# into upper right

plt.imshow(img2), plt.title('flipped corners')
plt.show()
