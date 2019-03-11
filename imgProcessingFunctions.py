import numpy as np
import cv2 as cv
import random as rng
import math
from scipy import interpolate

def getMarkerPositions(rawImg,centresImg):
    if rawImg.any() == None:
        print("No image Found\n")
        exit(1)

    hsvImg = cv.cvtColor(rawImg, cv.COLOR_BGR2HSV)
    lower_red = np.array([110,0,0])
    upper_red = np.array([120,255,255])
    filteredHsvImg = cv.inRange(hsvImg, lower_red, upper_red)

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
    areaThreshold = 50

    for cont in contours:
        if cv.contourArea(cont) > areaThreshold:
            areas.append(cv.contourArea(cont))
            momentsList.append(cv.moments(cont))

    centres = []
    distThreshold = 10
    finalCentresImg = np.zeros(rawImg.shape)
    #centresImg = rawImg.copy()
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
                cv.circle(finalCentresImg,tuple(tempCent), 5, (0,255,0), 5)
                #print("adding circle")

        else:
            centres.append(tempCent)
            cv.circle(finalCentresImg,tuple(tempCent), 5, (0,255,0), 5)
            #print("adding circle")
    #cv.imshow('finalCentresImg',finalCentresImg)

    #print("Found {} markers".format(len(centres)))
    return centres

def getRobotPositions(centres):
    robotPositions = np.array([]) # create numpy array to store robot positions
    maxabNorm = 50
    minabNorm = 10
    cDevThreshold = 10

    centresAllocated = np.zeros((1,len(centres)))

    centresAllocated = centresAllocated[0]

    for aIndex in range(len(centres)):
        if centresAllocated[aIndex] == 0: # ie if we havent allocated it yet
            a = centres[aIndex]
            for bIndex in range(len(centres)):
                if (bIndex != aIndex) and (centresAllocated[aIndex] == 0): # ie if we havent allocated it yet and its not 'a' candidate
                    b = centres[bIndex]
                    ab = b-a;
                    abNorm = np.linalg.norm(b-a)
                    if abNorm > maxabNorm or abNorm < minabNorm: # ie if its larger or smaller than expected
                        continue # ie begin loop on next b candidate
                    abOrth = np.array([-ab[1],ab[0]])
                    cEst0 = (0.5*ab+a) + 1.33*abOrth # get estimates for positions of third marker
                    cEst1 = (0.5*ab+a) - 1.33*abOrth
                    bestMatch = 0
                    bestMatchIndex = 0
                    smallestDist = 10000
                    for cIndex in range(len(centres)):
                        if cIndex != aIndex and bIndex != aIndex and centresAllocated[aIndex] == 0: # ie if we havent allocated it yet and its not 'a' or 'b' candidate
                            c = centres[cIndex]
                            if np.linalg.norm(c-cEst0) < smallestDist and  np.linalg.norm(c-cEst0) < cDevThreshold: # find the point closest to each estimate
                                smallestDist = np.linalg.norm(c-cEst0)
                                bestMatch = c
                                bestMatchIndex = cIndex

                            if np.linalg.norm(c-cEst1) < smallestDist and  np.linalg.norm(c-cEst1) < cDevThreshold:
                                smallestDist = np.linalg.norm(c-cEst1)
                                bestMatch = c
                                bestMatchIndex = cIndex

                    if smallestDist < 10000: # make sure one point has actually been selected
                        ab_c = np.array([[(0.5*ab+a)],[bestMatch]])

                        centroid = np.mean(ab_c,axis=0)[0]
                        ab2c = bestMatch-(0.5*ab+a)
                        #print("ab2c: {}".format(ab2c))
                        conv = np.array([[1,0],[0,1j]])
                        angle = np.angle(np.sum(np.matmul(ab2c,conv)),deg=False)
                        if robotPositions.shape == (0,):
                            robotPositions = np.array([centroid.item(0),centroid.item(1),angle])
                        else:
                            currentEntry = np.array([centroid.item(0),centroid.item(1),angle])
                            if robotPositions.shape == (3,):
                                temp_robotPositions = np.zeros((2,3))
                                temp_robotPositions[:-1,:] = robotPositions
                                temp_robotPositions[-1:,:] = currentEntry
                                robotPositions = temp_robotPositions
                            else:
                                rows,cols = robotPositions.shape
                                temp_robotPositions = np.zeros((rows,cols+1))
                                temp_robotPositions[:-1,:] = robotPositions
                                temp_robotPositions[-1:,:] = currentEntry
                                robotPositions = temp_robotPositions

                        centresAllocated[aIndex] = 1
                        centresAllocated[bIndex] = 1
                        centresAllocated[bestMatchIndex] = 1
    return robotPositions

def getObjectPerimeters(rawImg, pathPointResolution):
    hsvImg = cv.cvtColor(rawImg, cv.COLOR_BGR2HSV)
    lower = np.array([0,0,0])
    upper = np.array([255,255,10])
    filteredHsvImg = cv.inRange(hsvImg, lower, upper)
    filteredHsvImg = (255 - filteredHsvImg);
    kernel = np.ones((3,3),np.uint8)
    erodedImg = cv.erode(filteredHsvImg,kernel,iterations = 1)
    dilatedImg = cv.dilate(erodedImg,kernel,iterations = 3)
    dilatedImg = cv.blur(dilatedImg, (3,3)) # uncomment potentially?
    low_threshold = 0
    ratio = 3
    kernel_size = 3
    canny_edImg = cv.Canny(dilatedImg, low_threshold, low_threshold*ratio, kernel_size)
    contours, hierarchy = cv.findContours(canny_edImg, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
    contourImg = np.zeros(rawImg.shape)
    cv.drawContours(contourImg, contours, -1, (0,0,255), 3)

    hull_list = []
    for i in range(len(contours)):
        hull = cv.convexHull(contours[i])
        length,a,b = hull.shape
        hull = hull.reshape(length,b)
        xLast = hull.item((length-1,0))
        yLast = hull.item((length-1,1))
        hull_copy = hull.copy()
        index = 0
        for p in range(length):
            x = hull.item((p,0))
            y = hull.item((p,1))
            diffVect = np.array([x-xLast,y-yLast])
            if np.linalg.norm(diffVect) > pathPointResolution:
                print("Last x: {} and last y: {}".format(xLast,yLast))
                print("x: {} and y: {}".format(x,y))
                newPointsRequired = math.ceil(np.linalg.norm(diffVect) / pathPointResolution)
                newPoints = np.array([-1])
                ros = 0
                if abs(diffVect.item(0)) > abs(diffVect.item(1)): # ie if the x difference magnitude is greater than the y. this prevents singularities
                    print("X Mode")
                    xnew = np.arange(xLast, x, ((x-xLast)/newPointsRequired))
                    fx = interpolate.interp1d(np.array([xLast,x]), np.array([yLast,y]))
                    ynew = fx(xnew)
                    rows = xnew.shape[0]
                    newPoints = np.concatenate((xnew.reshape(rows,1),ynew.reshape(rows,1)))
                    newPoints = newPoints[1:,:]
                    print("newPoints")
                    print(newPoints)
                    print(" ")
                else:
                    print("Y Mode")
                    ynew = np.arange(yLast, y, ((y-yLast)/newPointsRequired))
                    fy = interpolate.interp1d(np.array([yLast,y]), np.array([xLast,x]))
                    xnew = fy(ynew)
                    rows = xnew.shape[0]
                    newPoints = np.concatenate((xnew.reshape(rows,1),ynew.reshape(rows,1)),axis=1)
                    newPoints = newPoints[1:,:]
                    print("newPoints")
                    print(newPoints)
                    print(" ")
                # by this point we have new points ready to insert
                print("###############pre-insertion::")
                print("#####hull_copy")
                print(hull_copy)
                print("#####newPoints")
                print(newPoints)
                hull_copy = np.insert(hull_copy, index,newPoints,axis=0)
                print("###############post-insertion::")
                print("#####hull_copy")
                print(hull_copy)
                index = index + rows
            index = index + 1
            xLast = x
            yLast = y
        hull_list.append(hull_copy)
        length,b = hull_copy.shape
        for p in range(length):
            x = hull_copy.item((p,0))
            y = hull_copy.item((p,1))
            cv.circle(rawImg,(x,y), 2, (0,255,0), 1)

    cv.imshow('rawImg',rawImg)
    return hull_list # finito!
