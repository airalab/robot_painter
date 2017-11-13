#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Oct 18 15:46:41 2017

@author: senex
"""

# Libs in PaleteFinder
# import rospy
# import tf2_ros
# import tf
# from cv_bridge import CvBridge, CvBridgeError
# from sensor_msgs.msg import Image
# from kuka_cv.msg import Palette
# from kuka_cv.msg import Colour
# from kuka_cv.srv import SetMode, SetModeResponse
# import cv2
from PaleteFinder import *

def main():
    rospy.init_node("kuka_cv")

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

    # TODO calculate scale factor, add function to calibration
    diffPaintDistanceM = [0.05, -0.012]                             # Distance between paints [m]
    diffPaintDistancePX = [66, -17]                                 # Distance between paints [px]
    scaleFactor = [diffPaintDistanceM[0]/diffPaintDistancePX[0],
                   diffPaintDistanceM[1]/diffPaintDistancePX[1]]    # Scale factor [m/px]

    # TODO add automatic calc of resolution
    resolution = [1280, 720]
    paletteThresh = 230
    freq = 10

    print("Waiting for set mode service.")
    detector = PaleteFinder(resolution, scaleFactor, paletteThresh, freq)
    try:
        rospy.spin()
    except CvBridgeError as e:
        print(e);

if __name__ == '__main__':
    main()