#!/usr/bin/python

'''
version of progmamme with servise for sending picture name
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
from picture_preprocessing.srv import TextConverterService, TextConverterServiceResponse
from kuka_cv.srv import *
from kuka_cv.msg import *
from PIL import Image, ImageDraw

rospack = rospkg.RosPack()
packagePath = rospack.get_path('picture_preprocessing') + "/"

PAPER_WIDTH = 0.1
PAPER_HEIGHT = 0.1
SMEAR_LENGTH = 200

BAG_FILE_PATH = packagePath + "data/"

def create_logo_pic(path_to_original_pic, path_to_logo):


    background = Image.open(packagePath + 'scripts/back.png')
    gaka = Image.open(packagePath + 'scripts/gaka.png')
    play = Image.open(packagePath + 'scripts/play.png')
    im = Image.open(path_to_original_pic)

    background.paste(im.resize((280, 280)), (230, 150))
    background.paste(gaka, (40, 250), gaka)
    background.paste(play, (int(background.size[0]/2-play.size[0]/2), \
                            int(background.size[1]/2-play.size[1]/2)), play)
    background.save(path_to_logo)

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

def convertText(req):
    while not rospy.is_shutdown():
        try:
            # Get information about canvas dimensions
            cnvsResp = canvasCient(1)
            PAPER_WIDTH = cnvsResp.width
            PAPER_HEIGHT = cnvsResp.height
            break;
        except rospy.ServiceException, e:
            print "Service call failed: {}".format(e)

    rospy.loginfo("Canvas dimensions: {0}x{1} m".format(PAPER_WIDTH, PAPER_HEIGHT))

    imageFile = req.data;
    #imageFile = '/home/kuka/kuka_pics/xrt.jpg'
    rospy.loginfo("path to picture: {0}".format(imageFile))
    if (not os.path.exists(imageFile)):
        rospy.logerr("Error: file '{}' does not exist!".format(imageFile))
        return False

    # create_logo_pic
    logo_name = packagePath + 'data/logo.png'
    create_logo_pic(imageFile, logo_name)


    src = cv2.imread(imageFile)
    src = cv2.rotate(src, cv2.ROTATE_90_COUNTERCLOCKWISE)

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
    #kx = ky = kw

    bw = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
    _, bw2 = cv2.threshold(bw, 10, 255, cv2.THRESH_BINARY_INV)
    bw2 = thinning(bw2)
    #cv2.imshow('bw2', bw2)
    #cv2.waitKey(0)

    im2, contours, hierarchy = cv2.findContours(bw2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
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
        plt.plot(x, y); #plt.show()

        ## Get trajectory from contour
        subcontours = []

        num_subcontours = len(contour)/SMEAR_LENGTH + 1
        for i in range(1, num_subcontours):
            subcontours.append(contour[(i-1)*SMEAR_LENGTH:i*SMEAR_LENGTH, 0])
        subcontours.append(contour[(num_subcontours-1)*SMEAR_LENGTH:len(contour), 0])

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

    return TextConverterServiceResponse()
if __name__ == "__main__":

    rospy.init_node("text_converter")
    textConverterService = rospy.Service("/convert_text", TextConverterService, convertText)
    canvasCient = rospy.ServiceProxy('/request_canvas', RequestCanvas)
    rospy.loginfo("Start image preprocessor!")

    while not rospy.is_shutdown():
        rospy.spin();
