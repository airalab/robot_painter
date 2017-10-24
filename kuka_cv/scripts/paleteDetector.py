#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Oct 18 15:46:41 2017

@author: senex
"""

import numpy as np
import cv2

cv2.namedWindow("thresh", cv2.WINDOW_NORMAL)
cv2.namedWindow("Raw", cv2.WINDOW_NORMAL)
# Load image in grayscale
img = cv2.imread("image.png")
grayimg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
grayimg = cv2.medianBlur(grayimg, 5) # 5 - kernel size

ret, thresh = cv2.threshold(grayimg, 250, 255, 0)
image, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, 
                                              cv2.CHAIN_APPROX_SIMPLE)
# hierarchy -- [Next, Previous, First_Child, Parent]

# Delete parant contour from hierarchy
for i in range(len(hierarchy)):
    if (hierarchy[0, i, 3] == -1):
        del contours[i]

# Draw all contours
img = cv2.drawContours(img, contours, -1, (0,0,0), 4)

# Find center of mass and color
colorPalete = []
for cnt in contours:
    M = cv2.moments(cnt)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    colorPalete.append(img[cy, cx])
    # cv2.circle(img, (cx, cy), 4, (0,0,0), -1)

print(colorPalete)
cv2.imshow("Raw", img)
cv2.imshow("thresh", thresh)
cv2.waitKey(0)
cv2.destroyAllWindows()