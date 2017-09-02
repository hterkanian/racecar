#!/usr/bin/python

import cv2
import numpy as np
# img = cv2.imread('Downloads/ml.jpg')
# print(img.shape)
a = np.array([[[1, 1, 2.5], [2,2,3]], [[3, 3, 2], [4,4,2.5]]])
print(a)
print('mean a axis 0:', np.mean(a, (0,1)))
print('mean a axis 1:', np.mean(a, (1,2)))
print('mean a axix 2:', np.mean(a, (2,0)))
c = np.flipud(a)
print(c)
d = np.fliplr(a)
print(d)
print('======')
b = a[:, 0]
print(b)
print('mean b axis 0:', np.mean(b,0))
print('mean b axis 1:', np.mean(b,1))

