#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Oct 18 15:46:41 2017

@author: senex
"""
import roslib
import rospy
import tf2_ros
import tf
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from kuka_cv.msg import Palette
from kuka_cv.msg import Colour

import numpy as np
import cv2

class PaleteFinder:

    def __init__(self, resolution, cameraPos, cameraRot, scaleFactor, paletteThresh, freq):
        self.bridge = CvBridge();
        self.imageSub = rospy.Subscriber("/Camera", Image, self.callback)
        self.palettePub = rospy.Publisher("/Palette", Palette, queue_size=10)
        self.font = cv2.FONT_HERSHEY_COMPLEX_SMALL

        self.freq = freq                    # Frquency of message sinding [hz]
        self.paletteThresh = paletteThresh  # Colour value of threshold for palette [0 - 255]

        self.width = resolution[0]          # camera resolution [px]
        self.height = resolution[1]         # camera resolution [px]
        self.dx = cameraPos.x               # camera in manipulator frame [m]
        self.dy = cameraPos.y               # camera in manipulator frame [m]
        self.dz = cameraPos.z               # camera in manipulator frame [m]
        self.rot = cameraRot                # camera Transformation 4x4 matrix
        self.kx = scaleFactor[0]            # scale factor [m/px]
        self.ky = scaleFactor[1]            # scale factor [m/px]

    def coordTransform(self, x, y, z):
        # First transform
        # Rotate camera frame to frame of joint_a6 (link_6)
        x1 = -(y - self.height/2)*self.kx
        y1 = (x - self.width/2)*self.ky
        z1 = z

        # print("p: " + str((x1, y1, z1)))
        # Second transform
        # Move to vector [dx, dy, dz]
        # Rotate camera frame to base frame (base_link)
        newX = self.rot[0, 0]*x1 + self.rot[0, 1]*y1 + self.rot[0, 2]*z1 + self.dx
        newY = self.rot[1, 0]*x1 + self.rot[1, 1]*y1 + self.rot[1, 2]*z1 + self.dy
        newZ = self.rot[2, 0]*x1 + self.rot[2, 1]*y1 + self.rot[2, 2]*z1

        return [round(newX, 4), round(newY, 4), round(newZ, 4)]

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

            coord = self.coordTransform(cx, cy, self.dz)

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

    # TODO add calibration mode
    calibration = True

    """
    If you don't know vector of camera relatibe manipulator frame set:
        
        cameraPosition.x = 0
        cameraPosition.y = 0
        cameraPosition.z = 0
        cameraOrientation.x = 0
        cameraOrientation.y = 0
        cameraOrientation.z = 0
        cameraOrientation.w = 1
        scaleFactor = [1, 1]

    After that by output data compute scale factor and set vector of camera.     
    """

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    tempRate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform("base_link", "camera_link", rospy.Time())
            print(trans)
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            tempRate.sleep()
            continue

    cameraPosition = trans.transform.translation;
    cameraRotation = trans.transform.rotation;

    rotMatrix = tf.transformations.quaternion_matrix([cameraRotation.x,
        cameraRotation.y,
        cameraRotation.z,
        cameraRotation.w])

    # TODO calculate scale factor, add function to calibration
    diffPaintDistanceM = [0.05, -0.012]                              # Distance between paints [m]
    diffPaintDistancePX = [66, -17]                                  # Distance between paints [px]
    scaleFactor = [diffPaintDistanceM[0]/diffPaintDistancePX[0],
                   diffPaintDistanceM[1]/diffPaintDistancePX[1]]    # Scale factor [m/px]

    # TODO add automatic calc of resolution
    resolution = [1280, 720]
    paletteThresh = 230
    freq = 10

    detector = PaleteFinder(resolution, cameraPosition, rotMatrix, scaleFactor, paletteThresh, freq)
    try:
        rospy.spin()
    except CvBridgeError as e:
        print(e);

if __name__ == '__main__':
    main()
