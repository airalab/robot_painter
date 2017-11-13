#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Oct 18 15:46:41 2017

@author: senex
"""

# import tf, tf2_ros, rospy
from CameraTransformations import *

# import OpenCV
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Messages
from sensor_msgs.msg import Image
from kuka_cv.msg import Palette
from kuka_cv.msg import Colour
from kuka_cv.srv import SetMode, SetModeResponse


### TODO add class for image processing
class ImageProcessing:
    def __init__(self, resolution, scaleFactor, tresh, baseFrameName, cameraFrameName):
        # OpenCV settings
        self.font = cv2.FONT_HERSHEY_COMPLEX_SMALL
        self.paletteThresh = tresh[0]   # Colour value of threshold for palette [0 - 255]
        self.canvasThres = tresh[1]     # Colour value of threshold for canvas [0 - 255]
        self.mode = 0                   # Mode of image processing

        # ROS API
        self.imageSub = rospy.Subscriber("/Camera", Image, imageCallback)
        self.palettePub = rospy.Publisher("/Palette", Palette, queue_size=10)
        self.modeService = rospy.Service("/SetPaleteDetectionMode", SetMode, setModeByService)
        self.freq = 10                  # Frquency of message sinding [hz]
        self.dataOutput = False

        # Enviroment information
        self.paletteMsg = Palette()

        # Frame Transformations; Need to set object vector
        self.transformer = CameraTransformations(resolution, scaleFactor, baseFrameName, cameraFrameName)
    
    ### Small class functions
    def setMode(self, mode):
        if (mode != 0 and mode != 1 and mode != 2):
            print("Mode error: setting mode is not correct. Please set mode (0, 1, 2)")
            return False

        self.mode = mode
        return True

    ### Common Image Processing
    def imageProcessing(self, img, grayimg):
        # Get camera position and orientation
        self.transformer.getCameraFrame()
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Load image in grayscale
        grayimg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        grayimg = cv2.medianBlur(grayimg, 5) # 5 - kernel size

        if (self.mode == 1 and not dataOutput):
            detectPalette()
        if (self.mode == 2 and not dataOutput):
            detectCanvas()

    def detectPalette(self, img, grayimg):
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
        self.paletteMsg = Palette() 
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

            self.paletteMsg.colours.append(colourMsg)
            self.dataOutput = True

        print(self.paletteMsg)

        cv2.imshow("Raw", img)
        cv2.imshow("thresh", thresh)
        cv2.waitKey(0)

        cv2.destroyAllWindows()

    def detectCanvas(self, img, grayimg):
        # Do somethin
        pass

    ### Callbacks
    def setModeByService(self, req):
        try: 
            resposnse = SetModeResponse(self.setMode(req.mode))
        except Exception:
            print("Exception")
        return resposnse


    def imageCallback(self, data):
        if self.mode = 0:
            return

        if not self.dataOutput:
            imageProcessing()
            return

        if mode == 1 and self.dataOutput:
            self.palettePub.publish(self.paletteMsg)

        self.rate.sleep()




def callback(data):


def main():
    rospy.init_node("kuka_cv")

    imageSub = rospy.Subscriber("/Camera", Image, imageCallback)
    palettePub = rospy.Publisher("/Palette", Palette, queue_size=10)
    modeService = rospy.Service("/SetPaleteDetectionMode", SetMode, setModeByService)

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
    thresholds = [230, 230]
    freq = 10

    # TODO detection
    print("Waiting for set mode service.")
    try:
        rospy.spin()
    except CvBridgeError as e:
        print(e);

if __name__ == '__main__':
    main()