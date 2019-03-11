#!/usr/bin/python3
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
cvpath = "/home/jamie/Libraries/opencv-4.0.1/build/lib"
import sys
import numpy as np
import cv2 as cv
from imgProcessingFunctions import *
print(cv.__version__)

################### get test image
inputImg = cv.imread('Screenshot4.jpg')
rows = 400
cols = 800
rawImg = cv.resize(inputImg,(cols, rows), interpolation = cv.INTER_CUBIC)

centresImg = np.zeros(rawImg.shape)
parametersImg = rawImg.copy()


####################### run functions
centres = getMarkerPositions(rawImg,centresImg)
#print("centres: {}".format(centres))

robotPositions = getRobotPositions(centres)

getObjectPerimeters2(parametersImg, 20)
#cv.imshow('invImage',img)

#################################### image demo rendering code:

#print("robotPositions")
#print(robotPositions)
if robotPositions.size != 0:
    if robotPositions.shape == (3,):
        baseNo = 0
        details = robotPositions.shape
        cv.circle(centresImg, (int(robotPositions[0]),int(robotPositions[1])), 5, ( 0, 0, 255 ), 1, 8 )
        length = 30;
        x = int(np.cos(robotPositions[2])*length)+int(robotPositions[0])
        print("x: {}".format(x))
        y = int(np.sin(robotPositions[2])*length)+int(robotPositions[1])
        print("y: {}".format(y))
        cv.arrowedLine(centresImg, (int(robotPositions[0]),int(robotPositions[1])), (x, y), (0,255,0), 2)

    else:
        print(size(robotPositions.shape))
        bases,details = robotPositions.shape
        for baseNo in range(bases):
            cv.circle(centresImg, (int(robotPositions.item(baseNo,0)),int(robotPositions.item(baseNo,1))), 5, ( 0, 0, 255 ), 1, 8 )
            length = 30;
            x = int(np.cos(robotPositions.item(baseNo,2))*length)+int(robotPositions.item(baseNo,0))
            print("x: {}".format(x))
            y = int(np.sin(robotPositions.item(baseNo,2))*length)+int(robotPositions.item(baseNo,1))
            print("y: {}".format(y))
            cv.arrowedLine(centresImg, (int(robotPositions.item(baseNo,0)),int(robotPositions.item(baseNo,1))), (x, y), (0,255,0), 2)

cv.imshow('centresImg',centresImg)


if cv.waitKey(0):# and 0xFF == ord('q'):
    cv.destroyAllWindows()
    exit()
