#!/usr/bin/python

import cv2
import numpy as np
img = cv2.imread('Downloads/ml.jpg')
print(img.shape)
img2 = cv2.imread('Downloads/messi_face.jpg')
img2_shape = img2.shape
print(img2.shape)
face = img2[0:img2_shape[0]-1, 0:img2_shape[1]-1]
img[20:face.shape[0]+20, 30:face.shape[1]+30] = face
cv2.imwrite('Downloads/ml_altered.jpg', img)

