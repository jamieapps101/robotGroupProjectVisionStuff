import numpy as np
import cv2 as cv
import random as rng

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
                print("adding circle")

        else:
            centres.append(tempCent)
            cv.circle(finalCentresImg,tuple(tempCent), 5, (0,255,0), 5)
            print("adding circle")
    #cv.imshow('finalCentresImg',finalCentresImg)

    print("Found {} markers".format(len(centres)))
    return centres

def getRobotPositions(centres):
    robotPositions = np.array([]) # create numpy array to store robot positions
    maxabNorm = 50
    minabNorm = 10
    cDevThreshold = 10

    centresAllocated = np.zeros((1,len(centres)))
    #print(centres)
    #print(centresAllocated)
    centresAllocated = centresAllocated[0]
    #print(centresAllocated)
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
                        print("ab2c: {}".format(ab2c))
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
    #rawImg = cv.blur(rawImg, (11,11)) # uncomment potentially?
    #cv.imshow('inputImg',rawImg)
    hsvImg = cv.cvtColor(rawImg, cv.COLOR_BGR2HSV)
    #cv.imshow('un-filteredHsvImg',hsvImg)
    lower_red = np.array([0,0,0])
    upper_red = np.array([255,255,10])
    filteredHsvImg = cv.inRange(hsvImg, lower_red, upper_red)
    #cv.imshow('filteredHsvImg',filteredHsvImg)
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
    #cv.imshow('contourImg',contourImg)
    #print(type(contours))
    #print("contours: {}".format(hierarchy))
    #print("hierarchy: {}".format(hierarchy))
    hull_list = []
    for i in range(len(contours)):
        hull = cv.convexHull(contours[i])
        hull_list.append(hull)
    print(type(hull_list[0]))

    drawing = np.zeros(rawImg.shape,dtype=np.uint8)
    amendedHull_list = []
    for h in hull_list:
        print(h.shape)
        points,height,coordNum = h.shape
        lastX = h.item(points-1,0,0)
        lastY = h.item(points-1,0,1)
        for p in range(points):
            x = h.item(p,0,0)
            y = h.item(p,0,1)
            cv.circle(drawing,(x,y), 2, (0,255,0), 1)
            if ((x-lastX)**2 + (y-lastY)**2)**0.5 > pathPointResolution:
                #find distvector
                # divide into equal units, where dist between < maxdistresolution
                # make new list of points
        #color = (rng.randint(0,256), rng.randint(0,256), rng.randint(0,256))
        #cv.drawContours(drawing, contours, i, color)
        #cv.drawContours(drawing, hull_list, i, color)
    #cv.imshow('drawing',drawing)



    return drawing
