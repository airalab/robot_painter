from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from kuka_cv.msg import Palette
from kuka_cv.msg import Colour
from kuka_cv.srv import SetMode, SetModeResponse

import cv2
from utils import *

class PaleteFinder:

    def __init__(self, resolution, scaleFactor, paletteThresh, freq):
        pos, rot = getCameraFrame()

        self.bridge = CvBridge();
        self.imageSub = rospy.Subscriber("/Camera", Image, self.callback)
        self.palettePub = rospy.Publisher("/Palette", Palette, queue_size=10)
        self.modeService = rospy.Service("/SetPaleteDetectionMode", SetMode, self.setModeByService)
        self.font = cv2.FONT_HERSHEY_COMPLEX_SMALL

        self.freq = freq                    # Frquency of message sinding [hz]
        self.rate = rospy.Rate(freq)
        self.paletteThresh = paletteThresh  # Colour value of threshold for palette [0 - 255]

        self.width = resolution[0]          # camera resolution [px]
        self.height = resolution[1]         # camera resolution [px]
        self.dx = pos.x                     # camera in manipulator frame [m]
        self.dy = pos.y                     # camera in manipulator frame [m]
        self.dz = pos.z                     # camera in manipulator frame [m]
        self.rot = rot                      # camera Transformation 4x4 matrix
        self.kx = scaleFactor[0]            # scale factor [m/px]
        self.ky = scaleFactor[1]            # scale factor [m/px]
        self.mode = 2                       # 2 - not working; 1 - measuring; 0 - sending;

        self.paleteMsg = Palette()

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

    def measuring(self, data):
        # Get camera position and orientation
        pos, rot = getCameraFrame()
        self.dx = pos.x
        self.dy = pos.y
        self.dz = pos.z

        self.rot = rot
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

        print(self.paletteMsg)

        cv2.imshow("Raw", img)
        cv2.imshow("thresh", thresh)
        cv2.waitKey(0)

        cv2.destroyAllWindows()

    def setMode(self, mode):
        if (mode != 2 and mode != 1 and mode != 0):
            print("Mode error: setting mode is not correct. Please set mode 0 or 1")
            return False

        self.mode = mode
        return True

    def setModeByService(self, req):
        try: 
            resposnse = SetModeResponse(self.setMode(req.mode))
        except Exception:
            print("Exception")
        return resposnse


    def callback(self, data):
        if self.mode == 2:
            return

        if self.mode == 1:
            self.measuring(data)
            self.setMode(0)

        if self.mode == 0:
            if (len(self.paletteMsg.colours) == 0):
                print("Cant set mode 0: Palette message size equal ZERO")
                return
            self.palettePub.publish(self.paletteMsg)
            self.rate.sleep()