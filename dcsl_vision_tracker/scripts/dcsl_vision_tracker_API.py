#!/usr/bin/env python

## @file
# This API allows the tracking of robots by one or multiple overhead cameras.
#
# It depends on OpenCV and the munkres library which is an implementation of the Hungarian algorithm.

## @author Brendan Andrade


import math as m
import cv
from munkres import Munkres




## This class is the superclass for overhead camera robot tracker classes. 
# 
# It provides a generic blob detection method and a method to match sensed robots with estimates of robot poses. It should likely not be used alone but as an inherited class for a tracking class specific to a system.
class DcslVisionTracker(object):

    ## Class initializer. This class needs no arguments or actions at initiation.
    def __init__(self):  
        pass

    ## Returns the poses of the robots in the image.
    #
    # Returns matched_poses (List of DcslPose objects) positions of 
    # @param image (CvMat) is the image in which to find the poses of the robots
    # @param estimated_poses (List of DcslPose objects) a guess of where the robots are located
    def get_poses(self, image, estimated_poses):

        # Find contours of blobs in the image
        contours = self.blob_contours(image, self.background, self.threshold, self.erodeIterations, self.storage)
        # Find poses of blobs in image reference frame
        image_poses = self.get_image_poses(contours)
        # Transform image frame coordinates to real world coordinates
        sensed_poses = self.coordinate_transform(image_poses, estimated_poses, camera_id = 0)
        # Match robots to estimates
        matched_poses = self.match_robots(sensed_poses, estimated_poses)
        return matched_poses
    

    ## Detects blobs in an image and returns the OpenCV contours of those blobs.
    #
    # @param image (CvMat) is the image in which to find contours. It should be a CvMat type.
    # @param backgroundMat (CvMat) is the background image. It should the image the camera sees when no robots are present.
    # @param binaryThreshold (int) is the value at which to threshold the image.
    # @param erodeIterations (int) is the number of iterations to erode the binary image. This eliminates small detected blobs.
    # @param storage (CvMemStorage) is a storage space for the contours.
    # @param maskMat (CvMat) is a binary mask. Areas to ignore should be black and all other space should be white.
    def blob_contours(self, image, backgroundMat, binaryThreshold, erodeIterations, storage, maskMat = None):
        '''
        if type(backgroundMat) is not cv.CvMat:
            raise Exception("Data type error: backgroundMat is not a CvMat")
        '''    
        imageMat = cv.CloneMat(image)
        
        #Subtract the background image
        cv.AbsDiff(imageMat,backgroundMat,imageMat)

        #Convert to binary image by thresholding
        cv.Threshold(imageMat, imageMat, binaryThreshold,255,cv.CV_THRESH_BINARY)

        #Apply mask
        if maskMat:
            cv.Min(imageMat, maskMat, imageMat); #mask image should be black (0) where masking should be applied and white (255) in the working area

        #Erode small holes in binary image
        cv.Erode(imageMat, imageMat, None, erodeIterations)

        #Find contours
        contours = cv.FindContours(imageMat, storage, cv.CV_RETR_LIST, cv.CV_CHAIN_APPROX_SIMPLE, (0,0))
        
        return contours
    

    ## Sorts the sensedPoses to match of the order of previousPose and detects whether or not each robot has been detected.
    #
    # Returns a list of DcslPose objects that contain sensed poses in the order of the previous poses. This allows detection of which robot is which.
    # Matching will fail if a false positive is detected in the same frame as a false negative (an undetected robot).
    # @param sensed_poses (list of DcslPose objects) the sensed positions of the robots from the image(s) in real world coordinates.
    # @param estimated_poses (list of DcslPose objects) the estimated positions of the robots in the order that is maintained across the system
    def match_robots(self, sensed_poses, estimated_poses):
        n_robots = len(estimated_poses)

        # Create a least squares error matrix between sensed poses (rows) and estimated poses (columns)
        cost_matrix = []
        for sensed_pose in sensed_poses:
            column = []
            for state in estimated_poses:
                difference = pow(sensed_pose.position_x()-state.position_x(),2)+pow(sensed_pose.position_y()-state.position_y(),2)
                column.append(difference)                
            cost_matrix.append(column)

        #Sort readings to match previous pose order
        sorted_measurements = []
        if len(cost_matrix) != 0: #There were sensed poses
            mun = Munkres()
            indexes = mun.compute(cost_matrix) #Returns a list of index pairs, 1st index cooresponds to the sensed pose and 2nd is the matched estimated pose
            for i in xrange(0,n_robots):
                assigned = False
                for pair in indexes:
                    if pair[1] == i: #Find estimated pose i in index pairs
                        sensed_poses_index = pair[0] #Matching sensed pose index is pair[0]
                        sensed_poses[sensed_poses_index].set_detected(True)
                        sorted_measurements.append(sensed_poses[sensed_poses_index])
                        assigned = True
                if not assigned: #If robot i was not found
                    empty_pose = DcslPose()
                    empty_pose.set_detected(False)
                    sorted_measurements.append(empty_pose)
        else: #No sensed poses
            for i in xrange(0,n_robots):
                empty_pose = DcslPose()
                empty_pose.set_detected(False)
                sorted_measurements.append(empty_pose)
        return sorted_measurements


#############################################################
                                              

class DcslMiabotTracker(DcslVisionTracker):

    def __init__(self, backgroundMat, binaryThreshold, erodeIterations, minBlobSize, maxBlobSize, scale, storage, image_width, image_height):
        DcslVisionTracker.__init__(self)
        self.background = backgroundMat
        self.threshold = binaryThreshold
        self.erodeIterations = erodeIterations
        self.minBlobSize = minBlobSize
        self.maxBlobSize = maxBlobSize
        self.scale = scale
        self.storage = storage
        self.image_width = image_width
        self.image_height = image_height
        

    ## Applies coordinate transform from image reference frame into real reference frame to image_poses and returns sensed_poses.
    #
    # Return sensed_poses is a list of DcslPose objects in a right hand coordinate system in meters and radians centered at the center of the image with x up in the image frame and y to the right. Theta measured CCW from x axis.
    # @param image_poses a list of poses in image (pixel) coordinates. Top left corner is the origin with the y axis down and the x axis right. Theta measured CCW from x axis.
    # @param estimated_poses (List of DcslPose objects) not required for miabot tracker.
    # @param camera_id (int) not required for miabot tracker.    
    def coordinate_transform(self, image_poses, estimated_poses = None, camera_id = None):
        sensed_poses = []
        for pose in image_poses:
            sensed_pose = DcslPose()
            sensed_x = (pose.position[0]-0.5*self.image_width)*self.scale
            sensed_y = -(pose.position[1]-0.5*self.image_height)*self.scale
            sensed_pose.set_position((sensed_x,sensed_y,0))
            sensed_pose.set_quaternion((None,None,pose.quaternion[2],None))
            sensed_poses.append(sensed_pose)
        return sensed_poses


    ## Takes the contours found in an image and returns the positions and headings of the blobs in image frame coordinates
    #
    # Returns image_poses (list of DcslPose objects) in the coordinate frame of the image of all contours in unknown order
    # @param contours (CvSeq of contours) contours of the blobs found in an image 
    def get_image_poses(self, contours):
        image_poses = [] # List to store measured poses
        cr = contours #Copy contours to not change original variable
        
        while cr is not None:
            # Find area of blob and reject those not the size of a robot
            blob_size = cv.ContourArea(cr)
            if blob_size > self.minBlobSize and blob_size < self.maxBlobSize:
                moments = cv.Moments(cr, False)
                # Find center of blob
                box = cv.MinAreaRect2(cr, cv.CreateMemStorage())
                center = (box[0][0], box[0][1])
                # Find centroid of blob
                centroid = (cv.GetSpatialMoment(moments,1,0)/cv.GetSpatialMoment(moments,0,0),cv.GetSpatialMoment(moments,0,1)/cv.GetSpatialMoment(moments,0,0))
                # Estimate heading by drawing line from centroid to center and finding heading of line
                theta_estimate = -m.atan2(center[1]-centroid[1],center[0]-centroid[0])
                # Theta is found using the minimum area box from above
                theta_box = -m.pi/180.0*box[2] #Angle in the first quadrant of the minimum area box with the x-axis
                # Test angle of box in each quadrant to see which is closest to angle found from centroid
                # Box angle is more accurate than centroid method
                previous_error = -1.0
                theta = None
                step_list = [-m.pi,-m.pi*0.5,0,m.pi*0.5]
                for step in step_list:
                    theta_test = theta_box+step
                    error = pow(theta_estimate-theta_test,2)
                    if previous_error < 0:
                        theta = theta_test
                    elif error < previous_error:
                        theta = theta_test
                    previous_error = error
                #Append found pose to image_poses list
                pose = DcslPose()
                pose.set_position((center[0],center[1],0))
                pose.set_quaternion((0,0,theta,0))
                image_poses.append(pose)
                del box
            #Cycle to next contour
            cr=cr.h_next()
        del cr 
        return image_poses
        
#############################################################################

class dcsl_beluga_tracker(DcslVisionTracker):
    
    def __init__(self):
        pass

#############################################################################

class DcslPose(object):
    def __init__(self):
        self.position = (None, None, None)
        self.quaternion = (None, None, None, None)
        self.detected = None
        self.robot_id = None
        
    def set_position(self, position):
        self.position = position

    def set_quaternion(self, quaternion):
        self.quaternion = quaternion
        
    def set_detected(self, is_detected):
        self.detected = is_detected

    def set_robot_id(self, robot_id):
        self.robot_id = robot_id

    def position_x(self):
        return self.position[0]
    
    def position_y(self):
        return self.position[1]
    
    def position_z(self):
        return self.position[2]
            
    def quaternion_x(self):
        return self.quaternion[0]

    def quaternion_y(self):
        return self.quaternion[1]

    def quaternion_z(self):
        return self.quaternion[2]

    def quaternion_w(self):
        return self.quaternion[3]
