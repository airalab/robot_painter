#!/usr/bin/python

'''
old version of picture_preprocessing without servise for sending picture name 
'''

import weave
import numpy as np
from imutils import resize
import matplotlib.pyplot as plt
import cv2
import rosbag
import os

import rospy
import rospkg
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseArray
import geometry_msgs.msg as gmsgs
from std_srvs.srv import Empty, EmptyResponse
from kuka_cv.srv import *
from kuka_cv.msg import *

rospack = rospkg.RosPack()
packagePath = rospack.get_path('picture_preprocessing') + "/"

PAPER_WIDTH = 0.1
PAPER_HEIGHT = 0.1
BAG_FILE_PATH = packagePath + "data/"

def _thinningIteration(im, iter):
    I, M = im, np.zeros(im.shape, np.uint8)
    expr = """
    for (int i = 1; i < NI[0]-1; i++) {
        for (int j = 1; j < NI[1]-1; j++) {
            int p2 = I2(i-1, j);
            int p3 = I2(i-1, j+1);
            int p4 = I2(i, j+1);
            int p5 = I2(i+1, j+1);
            int p6 = I2(i+1, j);
            int p7 = I2(i+1, j-1);
            int p8 = I2(i, j-1);
            int p9 = I2(i-1, j-1);

            int A  = (p2 == 0 && p3 == 1) + (p3 == 0 && p4 == 1) +
                     (p4 == 0 && p5 == 1) + (p5 == 0 && p6 == 1) +
                     (p6 == 0 && p7 == 1) + (p7 == 0 && p8 == 1) +
                     (p8 == 0 && p9 == 1) + (p9 == 0 && p2 == 1);
            int B  = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
            int m1 = iter == 0 ? (p2 * p4 * p6) : (p2 * p4 * p8);
            int m2 = iter == 0 ? (p4 * p6 * p8) : (p2 * p6 * p8);

            if (A == 1 && B >= 2 && B <= 6 && m1 == 0 && m2 == 0) {
                M2(i,j) = 1;
            }
        }
    }
    """

    weave.inline(expr, ["I", "iter", "M"])
    return (I & ~M)


def thinning(src):
    dst = src.copy() / 255
    prev = np.zeros(src.shape[:2], np.uint8)
    diff = None

    while True:
        dst = _thinningIteration(dst, 0)
        dst = _thinningIteration(dst, 1)
        diff = np.absolute(dst - prev)
        prev = dst.copy()
        if np.sum(diff) == 0:
            break

    return dst * 255

def convertText(data):

    while not rospy.is_shutdown():
        try:
            # Get information about canvas dimensions
            cnvsResp = canvasCient(1)
            PAPER_WIDTH = cnvsResp.width
            PAPER_HEIGHT = cnvsResp.height
            break;
        except rospy.ServiceException, e:
            print "Service call failed: {}".fromat(e)

    rospy.loginfo("Canvas dimensions: {0}x{1} m".format(PAPER_WIDTH, PAPER_HEIGHT))

    imageFile = "/home/kuka/kuka_pics/sign_30.jpg";
    rospy.loginfo(imageFile)
    if (not os.path.exists(imageFile)):
        rospy.logerr("Error: file '{}' does not exist!".format(imageFile))
        return False
    src = cv2.imread(imageFile)

    # Compute scale factor
    try:
        imgWidth, imgHeight, _ = src.shape
        rospy.loginfo("Image dimensions: {0}x{1} px".format(imgWidth, imgHeight))
    except Exception, e:
        rospy.logerr("Error: {}".format(e))
        return False

    kx = 0; ky = 0
    kw = float(PAPER_WIDTH)/imgWidth; kh = float(PAPER_HEIGHT)/imgHeight; # height and width coeffs
    kx = ky = kw if kh >= kw else kh


    bw = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
    _, bw2 = cv2.threshold(bw, 10, 255, cv2.THRESH_BINARY_INV)
    bw2 = thinning(bw2)
    cv2.imshow('bw2', bw2)
    cv2.waitKey(0)

    contours, hierarchy = cv2.findContours(bw2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(bw, contours, -1, (100,100,0), 2)

    # Create rosbag file
    bag = rosbag.Bag(BAG_FILE_PATH + 'test.bag', 'w')

    # Work with contours
    trajectorysNum = 0
    for i in xrange(len(contours)):
        contour = contours[i]
        sorted(contour, key = lambda point: point[0, 0])

        x = (contour[:, 0, 0] - imgWidth/2)*kx
        y = (imgHeight/2 - contour[:, 0, 1])*ky
        plt.plot(x, y); 
        # plt.show()

        ## Get trajectory from contour
        subcontours = []
        pi = 0
        prevCoord = contour[0, 0, 0]
        sign = np.sign(contour[0, 0, 1] - prevCoord)
        for i in xrange(1, contour.shape[0]):
            if (sign*(contour[i, 0, 0] - prevCoord) < 0):
                subcontours.append(contour[pi:i, 0])
                sign = np.sign(contour[i, 0, 0] - prevCoord)
                pi = i
            prevCoord = contour[i, 0, 0]

        # TODO filter subcontours by array size
	if contour.shape[0] == 2:
	    subcontours.append(contour[:, 0])

        size = len(subcontours)
        try:
            for num in xrange(size):
                trajectory = gmsgs.PoseArray()

                # simple filter
                if (subcontours[num].shape[0] <= 1):
                    # Debug information
                    # print("Shape {0}".format(subcontours[num].shape[0]))
                    # print(subcontours[num][:, 0]) # x
                    # print(subcontours[num][:, 1]) # y
                    continue

                x = (subcontours[num][:, 0] - imgWidth/2)*kx
                y = (imgHeight/2 - subcontours[num][:, 1])*ky
                plt.plot(x, y)
                # Fill trajectory message
                for p in xrange(x.shape[0]):
                    pose = gmsgs.Pose()
                    pose.position.x = x[p]
                    pose.position.y = y[p]

                    trajectory.poses.append(pose)
                trajectorysNum += 1
                bag.write('/path', trajectory)
        except Exception, e:
            print("Error: {}".format(e))

    bag.close()
    rospy.loginfo("Trajectory generated: {}".format(trajectorysNum))
    rospy.loginfo("All trajectorys was write to file {}".format(BAG_FILE_PATH + 'test.bag'))
    plt.show()
    return EmptyResponse()

if __name__ == "__main__":

    rospy.init_node("text_converter")
    textConverterService = rospy.Service("/convert_text", Empty, convertText)
    canvasCient = rospy.ServiceProxy('/request_canvas', RequestCanvas)
    rospy.loginfo("Start image preprocessor!")

    while not rospy.is_shutdown():
        rospy.spin();
