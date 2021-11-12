#!/usr/bin/python

'''
version of progmamme with servise for sending picture name
'''

import weave
import numpy as np
import matplotlib.pyplot as plt
import cv2
import os

PAPER_WIDTH = 0.1
PAPER_HEIGHT = 0.1
SMEAR_LENGTH = 200

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

def convertText():

    PAPER_WIDTH = 0.50
    PAPER_HEIGHT = 0.40

    print("Canvas dimensions: {0}x{1} m".format(PAPER_WIDTH, PAPER_HEIGHT))

    imageFile = 'res.png'
    print("path to picture: {0}".format(imageFile))
    if not os.path.exists(imageFile):
        print("Error: file '{}' does not exist!".format(imageFile))
        return False

    src = cv2.imread(imageFile)
    # src = cv2.rotate(src, cv2.ROTATE_90_COUNTERCLOCKWISE)

    # Compute scale factor
    imgWidth, imgHeight, _ = src.shape
    print("Image dimensions: {0}x{1} px".format(imgWidth, imgHeight))

    kx = 0; ky = 0
    kw = float(PAPER_WIDTH)/imgWidth; kh = float(PAPER_HEIGHT)/imgHeight; # height and width coeffs
    kx = ky = kw if kh >= kw else kh
    #kx = ky = kw

    bw = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
    _, bw2 = cv2.threshold(bw, 10, 255, cv2.THRESH_BINARY_INV)
    bw2 = thinning(bw2)
    cv2.imshow('bw2', bw2)
    cv2.waitKey(0)

    im2, contours, hierarchy = cv2.findContours(bw2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(bw, contours, -1, (100,100,0), 2)


    # Work with contours
    trajectorysNum = 0
    for i in xrange(len(contours)):
        contour = contours[i]
        sorted(contour, key = lambda point: point[0, 0])

        x = (contour[:, 0, 0] - imgWidth/2)*kx
        y = (imgHeight/2 - contour[:, 0, 1])*ky
        plt.plot(x, y);
        #plt.show()

        ## Get trajectory from contour
        subcontours = []

        num_subcontours = len(contour)/SMEAR_LENGTH + 1
        for i in range(1, num_subcontours):
            subcontours.append(contour[(i-1)*SMEAR_LENGTH:i*SMEAR_LENGTH, 0])
        subcontours.append(contour[(num_subcontours-1)*SMEAR_LENGTH:len(contour), 0])

        size = len(subcontours)
        try:
            for num in xrange(size):
                # simple filter
                if (subcontours[num].shape[0] <= 1):
                    continue

                x = (subcontours[num][:, 0] - imgWidth/2)*kx
                y = (imgHeight/2 - subcontours[num][:, 1])*ky
                plt.plot(x, y)
                trajectorysNum += 1
        except Exception, e:
            print("Error: {}".format(e))

    print("Trajectory generated: {}".format(trajectorysNum))
    plt.show()

if __name__ == "__main__":
    convertText()