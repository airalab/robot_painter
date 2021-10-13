#!/usr/bin/env python2

import json
from PIL import Image, ImageFilter

debugModeOn = True

makeSampleConf = False

# ROS
import rospy
import rospkg
from std_srvs.srv import Empty, EmptyResponse
from kuka_cv.srv import *
from kuka_cv.msg import *
import time

rospack = rospkg.RosPack()
packagePath = rospack.get_path('picture_preprocessing') + "/"
imagePalette = RequestPaletteResponse()

start = False;      # Bool variable for starting the work of node

""" Some general stuff goes here """

brushSize = 0.01

colors = []
canvas = []
size = []
dictToParse = {}

fond = (255, 255, 255)


def myRange(start, end, step):
    while start <= end:
        yield start
        start += step

def create_sample_conf():
    color_palette = {
                            "description": 42,
                            "version": 0.1,
                            "file": 'dao.jpg',
                            "colors": [
                                    {
                                        "color": "red",
                                        "ID": 0,
                                        "R": 255,
                                        "G": 0,
                                        "B": 0
                                    },
                                    {
                                        "color": "green",
                                        "ID": 1,
                                        "R": 0,
                                        "G": 255,
                                        "B": 0
                                    },{
                                        "color": "blue",
                                        "ID": 2,
                                        "R": 0,
                                        "G": 0,
                                        "B": 255
                                    },{
                                        "color": "yellow",
                                        "ID": 3,
                                        "R": 255,
                                        "G": 255,
                                        "B": 0
                                    },{
                                        "color": "cyan",
                                        "ID": 4,
                                        "R": 0,
                                        "G": 255,
                                        "B": 255
                                    },{
                                        "color": "magenta",
                                        "ID": 5,
                                        "R": 255,
                                        "G": 0,
                                        "B": 255
                                    },{
                                        "color": "black",
                                        "ID": 6,
                                        "R": 0,
                                        "G": 0,
                                        "B": 0
                                    }

                                ],
                            'height': 4,
                            'width': 2,
                            'brush': 0.05
                        }

    json.dump(color_palette, open(packagePath + 'sample_conf.json', 'w')) # save .json file

def load_sample_conf():
    with open(packagePath + 'sample_conf.json', 'r') as myfile: # open .json file
        raw_data=myfile.read().replace('\n', '')

    data = json.loads(raw_data) # load raw data from .json

    if (debugModeOn):
        print(raw_data) # debug stuff
        print(data["version"])

    palette = [] # color palette parsing
    for color in range( len(data["colors"]) ):
        palette += [[ data["colors"][color]['R'], data["colors"][color]['G'], data["colors"][color]['B'] ]]

    canvasDimentions = [data["width"], data["height"]] # get canvas Dimentions

    brush = data["brush"] # get brush size

    f = packagePath + data["file"] # get name of file with the picture to be drawed

    return brush, f

def choseColor(r, g, b, colors):

    marker = [] # create list of parameter that we want to minimize

    for i in range( len(colors) ): # calculating distances in color space from the point (r, g, b) to a points from color palette
        marker += [(r - colors[i][0]) ** 2 + (g - colors[i][1]) ** 2 + (b - colors[i][2]) ** 2]

    # print(marker);
    id = marker.index(min(marker), 0, len(marker)) # choose ID of the most simular color from our palette

    return id

def compose(d,l,canvas, brushSize):
    alpha, beta = d * brushSize / canvas[0], l * brushSize / canvas[1]

    print("Alpha: " + str(alpha))
    print("Beta:  " + str(beta))
    print("l: " + str(l))
    print("d: " + str(d))

    if ((alpha < 1) or (beta < 1)):
        return [0, 0]
    elif (alpha > beta):
        px = canvas[0] / brushSize
        return [int(px), int(float(l)/d * px)]
    else:
        px = canvas[1] / brushSize
        return [int(float(d)/l * px), int(px)]

def pixelatingPicture(img, Size):
    if (debugModeOn):
        print("Size A: " + str([img.size[0], img.size[1]]))
        print("Size B: " + str(Size))

    return img.resize(Size, Image.BILINEAR)

def exportData(w, h, pixels, colors, canvas, picture, brushSize):
    print("brushSize in export(): ", brushSize)

    dictToParse = {}

    dictToParse['test'] = 'is done'
    dictToParse['paintingPoints'] = []

    for i in range(len(colors)):
        for x in range(picture.size[0]):
            for y in range(picture.size[1]):
                if (pixels[x, y] != fond):
                    x_raw = (2*x  - 1) * brushSize/2  # convertation to pixels
                    y_raw = (2*y  - 1) * brushSize/2  # convertation to pixels

                    x_new = -round(x_raw - canvas[0]/2, 3)  # transform coordinates
                    y_new = -round(y_raw - canvas[1]/2, 3)  # transform coordinates

                    dictToParse['paintingPoints'].append({
                                                                'x': x_new,
                                                                'y': y_new,
                                                                'R': pixels[x, y][0],
                                                                'G': pixels[x, y][1],
                                                                'B': pixels[x, y][2]

                                                         })

                    # Create message with position of pixel center and BGR colour
                    colorMsg = Color(pixels[x, y][0], pixels[x, y][1], pixels[x, y][2])
                    poseMsg = Pose(x_new, y_new, 0, 0, 0, 0)
                    imagePalette.colors.append(colorMsg)
                    imagePalette.poses.append(poseMsg)

    # if (debugModeOn):
        # print(dictToParse) #debug stuff
        # print(imagePalette)
        # pass

    json.dump(dictToParse, open(packagePath + 'out.json', 'w')) # json dump

    print('Done')

# --------------------------------------- #

# TODO organize Action Server
""" Service Server for communication with LTP """
def startPreprocessing(data):

    print("READ palette and canvas message")
    colors = []
    canvas = []
    readMode = 1;
    while not rospy.is_shutdown():
        try:
            # Get information about colours
            clrResp = paletteClient(readMode)
            for clr in clrResp.colors:
                colors += [[ int(clr.r), int(clr.g), int(clr.b) ]]

            # Get information about canvas dimensions
            cnvsResp = canvasCient(readMode)
            canvas = [cnvsResp.width, cnvsResp.height]
            break;
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    print("Palette COLORS: " + str(colors))
    print("Canvas DIM: " + str(canvas))
    print("==========================================")

    # Start main image processing
    main(colors, canvas)

    resp = EmptyResponse()

    return resp

def sendImagePalette(data):
    while (len(imagePalette.colors) == 0 and not rospy.is_shutdown()):
        time.sleep(1)

    if rospy.is_shutdown():
        return False

    return imagePalette

def main(colors, canvas):
    if (makeSampleConf):
        create_sample_conf() #debug stuff

    """ Load environment data """
    brushSize, file = load_sample_conf()
    brushSize = 0.01;

    """ Load picture to be drawed """
    picture = Image.open(file)
    picSize = picture.size

    if (debugModeOn):
        picture.show() #debug stuff

    """ Pixelating """
    pixelSize = compose(picSize[0], picSize[1], canvas, brushSize) # compose() function checks if
                                                                            # picture needs to be resized
    print(pixelSize)

    if (pixelSize != [0, 0]):
        picture = pixelatingPicture(picture, pixelSize)
        picSize = picture.size

    if (debugModeOn):
        picture.show() #debug stuff
        print("Pixelated")

    """ Change colors of picture  """
    pixels = picture.load()

    for x in range(picture.size[0]):
        for y in range(picture.size[1]):
            if (pixels[x, y] != fond):
                i = choseColor(pixels[x, y][0], pixels[x, y][1], pixels[x, y][2], colors)
                pixels[x, y] = (colors[i][0], colors[i][1], colors[i][2])

    if (debugModeOn):
        picture.show() #debug stuff
        print("Colors has changed!")
        print("brushSize in main: ", brushSize)
    """ Export JSON file """

    exportData(picSize[0], picSize[1], pixels, colors, canvas, picture, brushSize)

if __name__ == '__main__':
    rospy.init_node('image_preprocessor')

    """ Start Servers and Clients """

    imagePeprocessingService = rospy.Service("/start_image_preprocessing", Empty, startPreprocessing)
    sendImagePaletteService = rospy.Service("/request_image_palette", RequestPalette, sendImagePalette)
    paletteClient = rospy.ServiceProxy('/request_palette', RequestPalette)
    canvasCient = rospy.ServiceProxy('/request_canvas', RequestCanvas)
    print("Waiting for Service.")
    while not rospy.is_shutdown():
        rospy.spin();
