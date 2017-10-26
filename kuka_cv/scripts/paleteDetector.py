#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Oct 18 15:46:41 2017

@author: senex
"""
import roslib
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import numpy as np
import cv2

class PaleteFinder:

    def __init__(self):
        self.bridge = CvBridge();
        self.imageSub = rospy.Subscriber("/Camera", Image, self.callback)
        self.font = cv2.FONT_HERSHEY_COMPLEX_SMALL
        self.kx = 0.131/1
        self.ky = 1/0.5

    def coordTransform(self, x, y):
        newX = y
        newY = x
        return (newX, newY)

    def callback(self, data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Load image in grayscale
        grayimg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        grayimg = cv2.medianBlur(grayimg, 5) # 5 - kernel size

        ret, thresh = cv2.threshold(grayimg, 230, 255, 0)
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
        palete = []
        paleteCoord = []
        for cnt in contours:
            M = cv2.moments(cnt)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            paleteColor = img[cy, cx]
            text = "(" + str(cx) + ", " + str(cy) + ")"
            cv2.putText(img, text, (cx, cy), self.font, 1, (0,0,255), 1, cv2.LINE_AA)

            paleteCoord.append(self.coordTransform(cx, cy))
            palete.append([paleteColor[0], paleteColor[1], paleteColor[2]])
            # cv2.circle(img, (cx, cy), 4, (0,0,0), -1)

        print(paleteCoord, palete)
        cv2.imshow("Raw", img)
        cv2.imshow("thresh", thresh)
        cv2.waitKey(0)

def main():
    rospy.init_node("palete_detector")

    # cv2.namedWindow("thresh", cv2.WINDOW_NORMAL)
    # cv2.namedWindow("Raw", cv2.WINDOW_NORMAL)

    detector = PaleteFinder()
    try:
        rospy.spin()
    except CvBridgeError as e:
        print(e);

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()