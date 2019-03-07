#!/usr/bin/python3
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
cvpath = "/home/jamie/Libraries/opencv-4.0.1/build/lib"
import sys
if ros_path in sys.path:

    sys.path.remove(ros_path)
    sys.path.append(cvpath)

#import cv2

import numpy as np
import cv2 as cv
print(cv.__version__)

inputImg = cv.imread('Screenshot.jpg')
rows = 400
cols = 800
rawImg = cv.resize(inputImg,(cols, rows), interpolation = cv.INTER_CUBIC)


if rawImg.any() == None:
    print("No image Found\n")
    exit(1)
else:
    print("Image Loaded")
cv.imshow('raw',rawImg)

hsvImg = cv.cvtColor(rawImg, cv.COLOR_BGR2HSV)
lower_red = np.array([0,220,85])
upper_red = np.array([255,255,255])
filteredHsvImg = cv.inRange(hsvImg, lower_red, upper_red)
#cv.imshow('filtered HSV',filteredHsvImg)

kernel = np.ones((3,3),np.uint8)
erodedImg = cv.erode(filteredHsvImg,kernel,iterations = 1)
dilatedImg = cv.dilate(erodedImg,kernel,iterations = 3)
# this could be repaced using "Opening" operation
#cv.imshow('erosion/dilation',dilatedImg)

dilatedImg = cv.blur(dilatedImg, (3,3)) # uncomment potentially?
low_threshold = 0
ratio = 3
kernel_size = 3
canny_edImg = cv.Canny(dilatedImg, low_threshold, low_threshold*ratio, kernel_size)
#cv.imshow('canny_edImg',canny_edImg)

contours, hierarchy = cv.findContours(canny_edImg, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
# where contours is a vector of a vector of points in c++
areas = []
momentsList = []
areaThreshold = 100

for cont in contours:
    if cv.contourArea(cont) > areaThreshold:
        areas.append(cv.contourArea(cont))
        momentsList.append(cv.moments(cont))

centres = []
distThreshold = 50
#centresImg = np.zeros(rawImg.shape)
centresImg = rawImg.copy()
for M in momentsList:
    tempCent = np.array([int(M['m10']/M['m00']),int(M['m01']/M['m00'])])
    toBeAdded = True
    #print("temp cent: {}".format(tempCent))
    if(len(centres) > 0):
        tempCentres = centres
        for c in tempCentres:
            if np.linalg.norm(tempCent - c) < distThreshold: # some centres are close due to duplication in moments calc, this removes them
                toBeAdded = False

        if toBeAdded == True:
            centres.append(tempCent)
            cv.circle(centresImg,tuple(tempCent), 10, (0,0,0), 1)

    else:
        centres.append(tempCent)
        cv.circle(centresImg,tuple(tempCent), 10, (0,0,0), 1)
cv.imshow('centresImg',centresImg)

 # we have now aquird the centres
 #we have now aquiredthe cent1es
 #we have now aquiredangles of all the tracking points
maxabDistVal = 200;
minabDistVal = 100;
maxcEstxCVal = 20;

robotPositions = np.array([]) # create numpy array to store robot positions

index = 0
for a in centres: # over veiw: for each centre
    if len(centres) < 3:
        break
    bCandidates = centres.copy()
    bCandidates.pop(index)
    index = index + 1
    bIndex = 0
    for b in bCandidates: # over veiw: pick a trial b candidate, work out where c should be
        if len(bCandidates) < 2:
            break;
        else:
            print("there are {} centres left".format(len(bCandidates)))
        cCandidates = bCandidates.copy()
        cCandidates.pop(bIndex)
        bIndex = bIndex + 1
        norm = np.linalg.norm(b-a)
        if norm < maxabDistVal and norm > minabDistVal: # ie is the dist between points roughly correct
            ab = b-a
            print("ab")
            print(ab)
            abOrth = np.array([-ab[1],ab[0]])
            print("abOrth")
            print(abOrth)
            cEst0 = (0.5*ab+a) + 1.044*abOrth # get estimates for positions of third marker
            cEst1 = (0.5*ab+a) - 1.044*abOrth
            print("cEst0")
            print(cEst0)
            print("cEst1")
            print(cEst1)
            bestMatch = 0
            smallestDist = 10000
            for c in cCandidates: # over veiw: test where the nearest point to c is
                if np.linalg.norm(c-cEst0) < smallestDist and  np.linalg.norm(c-cEst0) < maxcEstxCVal: # find the point closest to each estimate
                    smallestDist = np.linalg.norm(c-cEst0)
                    bestMatch = c

                if np.linalg.norm(c-cEst1) < smallestDist and  np.linalg.norm(c-cEst1) < maxcEstxCVal:
                    smallestDist = np.linalg.norm(c-cEst1)
                    bestMatch = c

           # over veiw: if a close c is found, assign it to a list
           #by this point we should have a,b and a good estimate of c
            if smallestDist < 10000: # make sure one point has actually been selected
                ab_c = np.array([[(0.5*ab+a)],[bestMatch]])
                centroid = np.mean(ab_c,axis=0)[0]
                print("Centroid: {}".format(centroid))
                conv = np.array([[1,0],[0,1j]])
                angle = np.angle(np.sum(np.matmul(a,conv)),deg=True)
                if robotPositions.shape == (0,):
                    #print(centroid.item(0))
                    #print(centroid.item(1))
                    print(angle)
                    robotPositions = np.array([centroid.item(0),centroid.item(1),angle])
                else:
                    currentEntry = np.array([centroid.item(0),centroid.item(1),angle])
                    robotPositions = np.append(robotPositions,currentEntry,axis=0)
                print("Centres before removal")
                print(centres)
                print("a")
                print(a)
                centres.remove(a)
                print("b")
                print(b)
                centres.remove(b)
                print("Centres after removal")
                print(centres)
                print("bestMatch")
                print(bestMatch)
                centres.remove(bestMatch)
                cv.circle(centresImg, (bestMatch.item(0),bestMatch.item(1)), 5, ( 0, 255, 0 ), 1, 8 )
                print("centres after all removed")
                print(centres)
                break

print("robotPositions")
print(robotPositions)
print("Shape")
print( robotPositions.shape)
if robotPositions.shape == (3,):
    bases = 0
    details = robotPositions.shape
    cv.circle(centresImg, (int(robotPositions.item(0)),int(robotPositions.item(1))), 5, ( 0, 0, 255 ))
else:
    bases,details = robotPositions.shape
    for baseNo in range(bases):
        cv.circle(centresImg, (robotPositions.item(baseNo,0),robotPositions.item(baseNo,1)), 5, ( 0, 0, 255 ), 1, 8 )
        print("Base numer {}".format(baseNo))


cv.imshow('centresImg',centresImg)

if cv.waitKey(0):# and 0xFF == ord('q'):
    cv.destroyAllWindows()
