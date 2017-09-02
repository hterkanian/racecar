#!/usr/bin/python

#video capture stub
# HST 7/23/17

import numpy as np
import cv2

cap = cv2.VideoCapture('Downloads/sample_video.mp4')

while( True ):
	ret, frame = cap.read()

	gray = cv2.cvtColor( frame, cv2.COLOR_BGR2GRAY )

	cv2.imshow( 'frame', gray )
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()
