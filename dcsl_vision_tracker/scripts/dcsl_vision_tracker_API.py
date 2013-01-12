#!/usr/bin/env python
import cv
from munkres import Munkres

class dcsl_vision_tracker(object):

    def __init__(self, firstImage, nRobots):
        if type(firstImage) is not cv.CvMat:
            raise Exception("Data type error: imageMat is not a CvMat")
        self.image = firstImage
        self.nRobots = nRobots

    def blobContours(self, backgroundMat, binaryThreshold, erodeIterations, storage):
        #Description
        #This function will change imageMat
        if type(backgroundMat) is not cv.CvMat:
            raise Exception("Data type error: backgroundMat is not a CvMat")

        imageMat = cv.CloneMat(self.image)
        
        #Subtract the background image
        cv.AbsDiff(imageMat,backgroundMat,imageMat)

        #Convert to binary image
        cv.Threshold(imageMat, imageMat, binaryThreshold)

        #Erode small holes in binary image
        cv.Erode(imageMat, imageMat, None, erodeIterations)

        #Find contours
        contours = cv.FindContours(imageMat, storage, cv.CV_RETR_LIST, cv.CV_CHAIN_APPROX_SIMPLE, (0,0))
        
        return contours

    def newImage(self, imageMat):
        if type(imageMat) is not cv.CvMat:
            raise Exception("Data type error: imageMat is not a CvMat")
        self.image = imageMat

    def matchRobots(self, readings, previousPoses):
        # This function sorts the readings from the image processing to match the order of the previous poses and returns the sorted list of tuples
        # The last entry in the pose indicts if the robot was sensed. 1 = sensed; 0 = not sensed
        # Create a least squares error matrix between readings and previous poses
        # Should be private
        costMatrix = []
        for reading in readings:
            temp = []
            for state in previousPoses:
                difference = pow(reading[0]-state.position.x,2)+pow(reading[1]-state.position.y,2)
                temp.append(difference)                
            costMatrix.append(temp)
        #Sort readings to match previous pose order
        sortedReadings = []
        if len(costMatrix) != 0:
            mun = Munkres()
            indexes = mun.compute(costMatrix)
            for i in xrange(0,self.nRobots):
                assigned = False
                for pair in indexes:
                    if pair[1] == i: 
                        readingsIndex = pair[0]
                        sortedReadings.append((readings[readingsIndex][0], readings[readingsIndex][1], readings[readingsIndex][2], 1))
                        assigned = True
                if not assigned:
                    sortedReadings.append((0,0,0,0))
        else:
            for i in xrange(0,self.nRobots):
                sortedReadings.append((0,0,0,0))
        return sortedReadings        
                                              

class dcsl_miabot_tracker(dcsl_vision_tracker):

    def __init__(self, firstImage, backgroundMat, binaryThreshold, erodeIterations, minBlobSize, maxBlobSize, scale, storage, nRobots):
        dcsl_vision_tracker.__init__(self, firstImage, nRobots)
        self.background = backgroundMat
        self.threshold = binaryThreshold
        self.erodeIterations = erodeIterations
        self.minBlobSize = minBlobSize
        self.maxBlobSize = maxBlobSize
        self.scale = scale
        self.storage = storage

    def poses(self, previousPoses):
        #Returns poses as list of floating point tuples of poses in order of previousPoses
        contours = self.blobContours(self.background, self.threshold, self.erodeIterations, self.storage)

        readings = [] #List to store measured poses

        while contours is not None:
            #Find area of blob and reject those not the size of a robot
            blobSize = cv.ContourArea(contours)
            if blobSize > self.minBlobSize and blobSize < self.maxBlobSize:
                moments = cv.Moments(contours, False)
                #Find center of robot
                box = cv.MinAreaRect2(contours, self.storage) #Maybe use temporary storage here
                center = (box[0][0], box[0][1])
                #Find centroid of blob
                centroid = (cv.GetSpatialMoment(moments,1,0)/cv.GetSpatialMoment(moments,0,0),cv.GetSpatialMoment(moments,0,1)/cv.GetSpatialMoment(moments,0,0))
                #Estimate heading
                thetaEstimate = -m.atan2(center[1]-centroid[1],center[0]-centroid[0])
                #Theta is found using the minimum area box from above
                thetaBox = -m.pi/180.0*box[2] #Angle of the minimum area box with the x-axis in the first quadrant
                previousError = -1.0
                theta = None
                stepList = [-m.pi,-m.pi*0.5,0,m.pi*0.5]
                for step in stepList:
                    thetaTest = thetaBox+step
                    error = pow(thetaEstimate-thetaTest,2)
                    if previousError < 0:
                        theta = thetaTest
                    elif error < previousError:
                        theta = thetaTest
                    previousError = error
                #Calculate position
                x = (center[0]-0.5*float(self.image.width))*self.scale
                y = -(center[1]-0.5*float(self.image.height))*self.scale #Minus sign because OpenCV uses left hand coordinate system
                readings.append((x,y,theta))
            #Cycle to next contour
            contours=contours.h_next()

        sortedReadings = self.matchRobots(readings, previousPoses)
        return sortedReadings


class dcsl_beluga_tracker(dcsl_beluga_tracker):
    
    def __init__(self):
        pass
