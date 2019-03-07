#!/usr/bin/python3
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
cvpath = "/home/jamie/Libraries/opencv-4.0.1/build/lib"
import sys
import numpy as np
import cv2 as cv
from imgProcessingFunctions import getMarkerPositions
from imgProcessingFunctions import getRobotPositions
#print(cv.__version__)

inputImg = cv.imread('Screenshot2.jpg')
rows = 400
cols = 800
rawImg = cv.resize(inputImg,(cols, rows), interpolation = cv.INTER_CUBIC)

centresImg = rawImg.copy()
centres = getMarkerPositions(rawImg,centresImg)

# Get a point from the "centres" list
#from every remaining point, check distances for max/min threshold, remove those not ideal
 #from remaining, create estimate points based on vectors
# for each point, test closeness to estimates. take closest
# check closest against threshold, accept if below
# from 3 points, find centroid and angle
# remove centroid from list

robotPositions = getRobotPositions(centres)

#################################### image demo rendering code:
print("robotPositions")
print(robotPositions)
if robotPositions.shape == (3,):
    bases = 0
    details = robotPositions.shape
    cv.circle(centresImg, (int(robotPositions.item(0)),int(robotPositions.item(1))), 5, ( 0, 0, 255 ))
else:
    bases,details = robotPositions.shape
    for baseNo in range(bases):
        cv.circle(centresImg, (int(robotPositions.item(baseNo,0)),int(robotPositions.item(baseNo,1))), 5, ( 0, 0, 255 ), 1, 8 )
        length = 30;
        x = int(np.cos(robotPositions.item(baseNo,2))*length)+int(robotPositions.item(baseNo,0))
        y = int(np.sin(robotPositions.item(baseNo,2))*length)+int(robotPositions.item(baseNo,1))
        cv.arrowedLine(centresImg, (int(robotPositions.item(baseNo,0)),int(robotPositions.item(baseNo,1))), (x, y), (0,255,0), 2)

cv.imshow('centresImg',centresImg)

if cv.waitKey(0):# and 0xFF == ord('q'):
    cv.destroyAllWindows()