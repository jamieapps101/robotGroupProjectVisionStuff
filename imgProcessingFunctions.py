import numpy as np
import cv2 as cv

def getMarkerPositions(rawImg,centresImg):
    if rawImg.any() == None:
        print("No image Found\n")
        exit(1)
    #else:
        #print("Image Loaded")
    #cv.imshow('raw',rawImg)


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
    areaThreshold = 80

    for cont in contours:
        if cv.contourArea(cont) > areaThreshold:
            areas.append(cv.contourArea(cont))
            momentsList.append(cv.moments(cont))

    centres = []
    distThreshold = 10
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
            #    cv.circle(centresImg,tuple(tempCent), 10, (0,0,0), 1)

        else:
            centres.append(tempCent)
            #cv.circle(centresImg,tuple(tempCent), 10, (0,0,0), 1)
    cv.imshow('centresImg',centresImg)

    print("Found {} markers".format(len(centres)))
    return centres

def getRobotPositions(centres):
    robotPositions = np.array([]) # create numpy array to store robot positions
    maxabNorm = 300
    minabNorm = 100
    cDevThreshold = 30

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
                    cEst0 = (0.5*ab+a) + 1.044*abOrth # get estimates for positions of third marker
                    cEst1 = (0.5*ab+a) - 1.044*abOrth
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
