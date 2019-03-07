#!/usr/bin/python3
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
cvpath = "/home/jamie/Libraries/opencv-4.0.1/build/lib"
import sys
import numpy as np
import cv2 as cv
#from imgProcessingFunctions import getMarkerPositions
#from imgProcessingFunctions import getRobotPositions
#from imgProcessingFunctions import getObjectPerimeters
from imgProcessingFunctions import *
print(cv.__version__)

inputImg = cv.imread('Screenshot4.jpg')
rows = 400
cols = 800
rawImg = cv.resize(inputImg,(cols, rows), interpolation = cv.INTER_CUBIC)
#cv.imshow('rawImg',rawImg)
centresImg = np.zeros(rawImg.shape)
parametersImg = rawImg.copy()
#centresImg = rawImg.copy()
centres = getMarkerPositions(rawImg,centresImg)
print("centres: {}".format(centres))


# Get a point from the "centres" list
#from every remaining point, check distances for max/min threshold, remove those not ideal
 #from remaining, create estimate points based on vectors
# for each point, test closeness to estimates. take closest
# check closest against threshold, accept if below
# from 3 points, find centroid and angle
# remove centroid from list

robotPositions = getRobotPositions(centres)

cv.imshow('invImage',getObjectPerimeters(parametersImg, 10))

#################################### image demo rendering code:

print("robotPositions")
print(robotPositions)
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

#cv.imshow('centresImg',centresImg)


if cv.waitKey(0):# and 0xFF == ord('q'):
    cv.destroyAllWindows()
    exit()
