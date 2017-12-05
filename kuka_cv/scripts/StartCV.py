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
from geometry_msgs.msg import Transform
from kuka_cv.msg import Palette
from kuka_cv.msg import Colour
from kuka_cv.srv import RequestPalette, RequestPaletteResponse
from kuka_cv.srv import RequestCanvas, RequestCanvasResponse
from kuka_cv.srv import SetMode, SetModeResponse

# Othre
import numpy as np


### TODO add class for image processing
class ImageProcessing:
    def __init__(self, resolution, scaleFactor, tresh, baseFrameName, cameraFrameName, freq):
        self.bridge = CvBridge();

        # OpenCV settings
        self.font = cv2.FONT_HERSHEY_COMPLEX_SMALL
        self.paletteThresh = tresh[0]   # Colour value of threshold for palette [0 - 255]
        self.canvasThresh = tresh[1]    # Colour value of threshold for canvas [0 - 255]
        self.mode = 0                   # Mode of image processing

        # ROS API
        self.imageSub = rospy.Subscriber("/Camera", Image, self.imageCallback)
        self.paletteService = rospy.Service("/request_palette", RequestPalette, self.sendPaletteInfo)
        self.canvasService = rospy.Service("/request_canvas", RequestCanvas, self.sendCanvasInfo)
        self.modeService = rospy.Service("/SetPaleteDetectionMode", SetMode, self.setModeByService)
        self.freq = freq                # Frquency of message sinding [hz]
        self.rate = rospy.Rate(self.freq)
        self.computeImage = False

        # Enviroment information
        self.paletteMsg = Palette()
        self.canvasTranform = Transform()
        self.canvasW = 0
        self.canvasH = 0
        self.canvasRot = 0

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
    def imageProcessing(self, data):
        # Get camera position and orientation
        print("Get current frame information.")
        self.transformer.getCameraFrame()
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Load image in grayscale
        grayimg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        grayimg = cv2.medianBlur(grayimg, 5) # 5 - kernel size

        if (self.mode == 1):
            print("[CV] Detect plette!")
            self.paletteMsg = Palette()
            self.detectPalette(img, grayimg)
            self.computeImage = False
            return
        if (self.mode == 2):
            print("[CV] Detect canvas!")
            self.canvasTranform = Transform()
            self.detectCanvas(img, grayimg)
            self.computeImage = False
            return

    def detectPalette(self, img, grayimg):
        msg = Palette()
        ret, thresh = cv2.threshold(grayimg, self.paletteThresh, 255, 0)
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
        # TODO try to use cv2.mean(image, mask)
        for cnt in contours:
            M = cv2.moments(cnt)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            paleteColor = img[cy, cx]
            # TODO Add z coordinate as param
            coord = self.transformer.coordTransform(cx, cy, 0)

            colourMsg = Colour()
            colourMsg.position = [coord[0], coord[1], coord[2]]
            colourMsg.bgr = [paleteColor[0], paleteColor[1], paleteColor[2]]

            text = "(" + str(round(coord[3], 3)) + ", " + str(round(coord[4], 3)) + ")"
            cv2.putText(img, text, (cx, cy), self.font, 1, (0,0,255), 1, cv2.LINE_AA)
            # cv2.circle(img, (cx, cy), 4, (0,0,0), -1)

            msg.colours.append(colourMsg)

        cv2.imshow("Raw", img)
        cv2.imshow("thresh", thresh)
        cv2.waitKey(0)

        cv2.destroyAllWindows()

        self.paletteMsg = msg
        print(self.paletteMsg)

    def detectCanvas(self, img, grayimg):
        # Get camera position and orientation
        print("Get current frame information.")
        self.transformer.getCameraFrame()

        ret, thresh = cv2.threshold(grayimg, self.canvasThresh, 255, 0)
        image, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE,
                                                    cv2.CHAIN_APPROX_SIMPLE)
        rect = cv2.minAreaRect(contours[0])
        box = cv2.boxPoints(rect)
        cx, cy = np.int0(rect[0])
        w, h = np.int0(rect[1])

        ang = -rect[2]              # Angle between vector x and canvas TODO check angle
        if (abs(ang - 2) > 45):
            ang = 90 - np.sign(ang)*ang

        print("angle: " + str(ang))
        coord = self.transformer.coordTransform(cx, cy, 0)      # TODO add Z
        q1 = tf.transformations.quaternion_from_euler(0, 0, ang)
        q2 = self.transformer.q
        q = tf.transformations.quaternion_multiply(q1, q2)
        print("q1: " + str(q1))

        # Draw all contours
        img = cv2.drawContours(img, contours, -1, (0,0,0), 4)

        cv2.imshow("Raw", img)
        cv2.imshow("thresh", thresh)
        cv2.waitKey(0)

        cv2.destroyAllWindows()

        self.canvasTranform.translation.x = coord[0]
        self.canvasTranform.translation.y = coord[1]
        self.canvasTranform.translation.z = coord[2]
        self.canvasTranform.rotation.x = q[0]
        self.canvasTranform.rotation.y = q[1]
        self.canvasTranform.rotation.z = q[2]
        self.canvasTranform.rotation.w = q[3]
        self.canvasW = np.sqrt((w*np.sin(ang)*self.transformer.kx)**2 + (w*np.cos(ang)*self.transformer.ky)**2)
        self.canvasH = np.sqrt((h*np.cos(ang)*self.transformer.kx)**2 + (h*np.sin(ang)*self.transformer.ky)**2)

    ### Callbacks
    def setModeByService(self, req):
        try:
            self.computeImage = True
            resposnse = SetModeResponse(self.setMode(req.mode))
            if resposnse:
                print("Set mode: " + str(self.mode))
            else:
                print("Error: mode is not set")

        except Exception:
            print("Exception")
        return resposnse

    def sendPaletteInfo(self, req):
        resp = RequestPaletteResponse()
        while len(self.paletteMsg.colours) == 0:
            self.rate.sleep()
        resp.colours = self.paletteMsg.colours
        return resp

    def sendCanvasInfo(self, req):
        resp = RequestCanvasResponse()
        check = (self.canvasTranform.translation.x == 0
            and self.canvasTranform.translation.y == 0
            and self.canvasTranform.translation.z == 0)
        while (check):
            check = (self.canvasTranform.translation.x == 0
                and self.canvasTranform.translation.y == 0
                and self.canvasTranform.translation.z == 0)
            self.rate.sleep()

        resp.trans = self.canvasTranform
        resp.width = self.canvasW
        resp.height = self.canvasH
        return resp


    def imageCallback(self, data):
        if self.mode == 0:
            return

        if self.computeImage:
            print("Image Prosessing start.")
            self.imageProcessing(data)
            return

        self.rate.sleep() # TODO delete rate


def main():
    rospy.init_node("kuka_cv")

    # TODO add calibration mode
    calibration = True

    """
    If you don't know vector of camera relatibe manipulator frame set:

        scaleFactor = [1, 1]

    After that by output data compute scale factor and set vector of camera.     
    """

    # TODO calculate scale factor, add function to calibration
    diffPaintDistanceM = [0.0501, -0.013]                           # Distance between paints [m]
    diffPaintDistancePX = [66, -17]                                 # Distance between paints [px]
    scaleFactor = [diffPaintDistanceM[0]/diffPaintDistancePX[0],
                   diffPaintDistanceM[1]/diffPaintDistancePX[1]]    # Scale factor [m/px]

    # TODO add automatic calc of resolution
    resolution = [1280, 720]
    thresholds = [230, 230]
    freq = 10

    # TODO detection
    print("Waiting for set mode service.")
    imageProcessor = ImageProcessing(resolution, scaleFactor, thresholds, "base_link", "camera_link", freq)

    try:
        rospy.spin()
    except CvBridgeError as e:
        print(e);

if __name__ == '__main__':
    main()