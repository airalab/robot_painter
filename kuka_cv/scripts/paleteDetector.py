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
from kuka_cv.msg import Palette
from kuka_cv.msg import Colour

import numpy as np
import cv2

class PaleteFinder:

    def __init__(self, resolution, vectorOfCamera, scaleFactor, paletteThresh, freq):
        self.bridge = CvBridge();
        self.imageSub = rospy.Subscriber("/Camera", Image, self.callback)
        self.palettePub = rospy.Publisher("/Palette", Palette, queue_size=10)
        self.font = cv2.FONT_HERSHEY_COMPLEX_SMALL

        self.freq = freq                    # Frquency of message sinding [hz]
        self.paletteThresh = paletteThresh  # Colour value of threshold for palette [0 - 255]

        self.width = resolution[0]          # camera resolution [px]
        self.height = resolution[1]         # camera resolution [px]
        self.dx = vectorOfCamera[0]         # camera in manipulator frame [m]
        self.dy = vectorOfCamera[1]         # camera in manipulator frame [m]
        self.kx = scaleFactor[0]            # scale factor [m/px]
        self.ky = scaleFactor[1]            # scale factor [m/px]

    def coordTransform(self, x, y, z):
        newX = (-x + self.width/2)*self.kx + self.dx
        newY = (-y + self.height/2)*self.ky + self.dy
        newZ = z
        return [round(newX, 4), round(newY, 4), newZ]

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
        paletteMsg = Palette() 
        for cnt in contours:
            M = cv2.moments(cnt)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            paleteColor = img[cy, cx]

            coord = self.coordTransform(cx, cy, 0)

            colourMsg = Colour()
            colourMsg.position = coord
            colourMsg.bgr = [paleteColor[0], paleteColor[1], paleteColor[2]]

            text = "(" + str(coord[0]) + ", " + str(coord[1]) + ")"
            cv2.putText(img, text, (cx, cy), self.font, 1, (0,0,255), 1, cv2.LINE_AA)
            # cv2.circle(img, (cx, cy), 4, (0,0,0), -1)

            paletteMsg.colours.append(colourMsg)

        print(paletteMsg)

        cv2.imshow("Raw", img)
        cv2.imshow("thresh", thresh)
        cv2.waitKey(0)

        cv2.destroyAllWindows()

        rate = rospy.Rate(self.freq)     # freq [hz]
        while not rospy.is_shutdown():
            self.palettePub.publish(paletteMsg)
            rate.sleep()

def main():
    rospy.init_node("palete_detector")

    """
    If you don't know vector of camera relatibe manipulator frame set:
        
        resulotion = [0, 0]
        vectorOfCamera = [0, 0]
        scaleFactor = [1, 1]

    After that by output data compute scale factor and set vector of camera.     
    """

    resolution = [1280, 720]
    vectorOfCamera = [0.0925, 0.5]
    scaleFactor = [0.014/16, 0.05/57]
    paletteThresh = 230
    freq = 10

    detector = PaleteFinder(resolution, vectorOfCamera, scaleFactor, paletteThresh, freq)
    try:
        rospy.spin()
    except CvBridgeError as e:
        print(e);

if __name__ == '__main__':
    main()