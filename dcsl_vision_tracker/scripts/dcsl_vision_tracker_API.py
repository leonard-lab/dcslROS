#!/usr/bin/env python

## @file
# This API allows the tracking of robots by one or multiple overhead cameras.
#
# It depends on OpenCV and the munkres library which is an implementation of the Hungarian algorithm.

## @author Brendan Andrade


import math as m
import cv
from munkres import Munkres


##################################################


## This class is the superclass for overhead camera robot tracker classes. 
# 
# It provides a generic blob detection method and a method to match sensed robots with estimates of robot poses. It should likely not be used alone but as an inherited class for a tracking class specific to a system.
# The subclass must have methods get_image_poses and coordinate_transform as defined in this class.
class DcslVisionTracker(object):

    ## Class initializer. Loads arguments into object variables.
    #
    # @param background_list A list of background images. The ith list entry should be the background for camera id i.
    # @param mask_list A list of masks. The ith list entry should be the mask for camera id i.
    # @param threshold  The cut off brightness value for converting from a gray scale to binary image.
    # @param erode_iterations The number of times to erode the binary image. This is used to eliminate small detected blobs. More iterations removes larger blobs.
    # @param min_blob_size The minimum expected blob size in square pixels for a robot.
    # @param max_blob_size The maximum expected blob size in square pixels for a robot.
    # @param storage An OpenCV storage container created with cv.CreateMemStorage.
    # @param image_width Width of the image to be tracked in pixels.
    # @param image_height Height of the image to be tracked in pixels.
    def __init__(self, background_list, mask_list, binary_threshold, erode_iterations, min_blob_size, max_blob_size, storage, image_width, image_height):  
        self.background_list = []
        # Convert the given to grayscale in the same way the tracker does. It's better to do it this way than pre-convert.
        for background in background_list:
            gray_background = self.convert_to_grayscale(background)
            self.background_list.append(gray_background)
        self.mask_list = mask_list
        self.threshold = binary_threshold
        self.erode_iterations = erode_iterations
        self.min_blob_size = min_blob_size
        self.max_blob_size = max_blob_size
        self.storage = storage
        self.image_width = image_width
        self.image_height = image_height

        ## @var background
        # A list of background images. The ith list entry should be the background for camera id i.
        
        ## @var mask_list
        # A list of masks. The ith list entry should be the mask for camera id i.
        
        ## @var threshold
        # The cut off brightness value for converting from a gray scale to binary image.

        ## @var erode_iterations
        # The number of times to erode the binary image. This is used to eliminate small detected blobs. More iterations removes larger blobs.

        ## @var min_blob_size
        # The minimum expected blob size in square pixels for a robot.
        
        ## @var max_blob_size
        # The maximum expected blob size in square pixels for a robot.
        
        ## @var storage
        # An OpenCV storage container created with cv.CreateMemStorage.
        
        ## @var image_width
        # Width of the image to be tracked in pixels.

        ## @var image_height
        # Height of the image to be tracked in pixels.

    ## Returns the poses of the robots in the image.
    #
    # Returns matched_sensed_world (List of DcslPose objects) positions of robots in real world frame in the same order as estimated_poses
    # Returns matched_sensed_image (List of DcslPose objects) positions of robots in image coordinates in the same order as estimated_poses
    # Returns contours (CvSeq) structure containing the contours of blobs found in the image
    # @param image (CvMat) is the image in which to find the poses of the robots
    # @param estimated_poses (List of DcslPose objects) a guess of where the robots are located
    # @param camera_id (int) the number of the camera the image comes from. This is used in the coordinate_transform function to chose the correct transform.
    def get_poses(self, image, estimated_poses, camera_id):
        
        #Convert to greyscale
        grayscale_image = self.convert_to_grayscale(image)
        # Find contours of blobs in the image
        contours = self.blob_contours(grayscale_image, self.background_list[camera_id], self.threshold, self.erode_iterations, self.storage)
        # Find poses of blobs in image reference frame
        sensed_image = self.get_image_poses(contours)
        # Transform estimated poses to image frame
        estimated_image = self.world_to_image(estimated_poses, camera_id)
        # Match robots to estimates in image frame
        matched_sensed_image = self.match_robots(sensed_image, estimated_image)
        # Transform image frame coordinates to real world coordinates
        matched_sensed_world = self.image_to_world(matched_sensed_image, estimated_poses, camera_id)
        return matched_sensed_world, matched_sensed_image, contours
    
    ## Detects blobs in an image and returns the OpenCV contours of those blobs.
    #
    # @param image (CvMat) is the image in which to find contours. It should be a CvMat type.
    # @param backgroundMat (CvMat) is the background image. It should the image the camera sees when no robots are present.
    # @param binaryThreshold (int) is the value at which to threshold the image.
    # @param erodeIterations (int) is the number of iterations to erode the binary image. This eliminates small detected blobs.
    # @param storage (CvMemStorage) is a storage space for the contours.
    # @param maskMat (CvMat) is a binary mask. Areas to ignore should be black and all other space should be white.
    def blob_contours(self, image, background_mat, binary_threshold, erode_iterations, storage, mask_mat = None):
        
        image_mat = cv.CloneMat(image)
        
        # Subtract the background image
        cv.AbsDiff(image_mat, background_mat, image_mat)

        # Convert to binary image by thresholding
        cv.Threshold(image_mat, image_mat, binary_threshold, 255, cv.CV_THRESH_BINARY)

        # Apply mask
        if mask_mat:
            cv.Min(image_mat, mask_mat, image_mat); # mask image should be black (0) where masking should be applied and white (255) in the working area

        # Erode small holes in binary image
        cv.Erode(image_mat, image_mat, None, erode_iterations)

        # Dilate remaining holes to offset erode
        cv.Dilate(image_mat, image_mat, None, erode_iterations)

        #Find contours
        contours = cv.FindContours(image_mat, storage, cv.CV_RETR_LIST, cv.CV_CHAIN_APPROX_SIMPLE, (0,0))
        
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

        # Sort readings to match previous pose order
        sorted_measurements = []
        if len(cost_matrix) != 0: # There were sensed poses
            mun = Munkres()
            indexes = mun.compute(cost_matrix) # Returns a list of index pairs, 1st index cooresponds to the sensed pose and 2nd is the matched estimated pose
            for i in xrange(0,n_robots):
                assigned = False
                for pair in indexes:
                    if pair[1] == i: # Find estimated pose i in index pairs
                        sensed_poses_index = pair[0] #Matching sensed pose index is pair[0]
                        sensed_poses[sensed_poses_index].set_detected(True)
                        sorted_measurements.append(sensed_poses[sensed_poses_index])
                        assigned = True
                if not assigned: # If robot i was not found
                    empty_pose = DcslPose()
                    empty_pose.set_detected(False)
                    sorted_measurements.append(empty_pose)
        else: # No sensed poses
            for i in xrange(0,n_robots):
                empty_pose = DcslPose()
                empty_pose.set_detected(False)
                sorted_measurements.append(empty_pose)
        return sorted_measurements

    ## Applies coordinate transform from image reference frame into real reference frame to image_poses and returns sensed_poses.
    #
    # This function needs to be defined in the subclass.
    # Return sensed_poses is a list of DcslPose objects in a right hand coordinate system in meters and radians centered at the center of the image with x up in the image frame and y to the right. Theta measured CCW from x axis.
    # @param image_poses a list of poses in image (pixel) coordinates. Top left corner is the origin with the y axis down and the x axis right. Theta measured CCW from x axis.
    # @param estimated_poses (List of DcslPose objects) is required if the transform depends on the state of the robot.
    # @param camera_id (int) is uses to apply for the correct transform in the case of multi-camera systems.
    def image_to_world(self, image_poses, estimated_poses, camera_id):
        pass

    ## Takes the contours found in an image and returns the positions and headings of the blobs in image frame coordinates.
    #
    # This function needs to be defined in the subclass.
    # Returns image_poses (list of DcslPose objects) in the coordinate frame of the image of all contours in unknown order
    # @param contours (CvSeq of contours) contours of the blobs found in an image 
    def get_image_poses(self, contours):
        pass

    ## Applies coordinate transform from world frame to image frame on world_poses and returns image_poses.
    #
    # This function needs to be defined in the subclass.
    # Return image_poses is a list of DcslPose objects in image (pixel) coordinates. Top left corner is the origin with the y axis down and the x axis right. Theta is measured CCW from x axis.
    # @param world_poses a list of poses in the world reference frame (meters and radians).
    # @param camera_id (int) is uses to apply for the correct transform in the case of multi-camera systems.
    def world_to_image(self, world_poses, camera_id):
        pass

    ## Converts the image acquired from the camera to an 8-bit cvMat grayscale image.
    #
    # This function needs to be defined in the subclass if the camera is not receiving 8-bit cvMat images.
    # @param image An image to be converted to grayscale.
    # Return grayscale_image an 8-bit cvMat image.
    def convert_to_grayscale(self, image):
        grayscale_image = cv.CloneMat(image)
        return grayscale_image


#############################################################
                                              

class DcslMiabotTracker(DcslVisionTracker):

    ## Class initializer. Loads arguments into object variables.
    #
    # @param background_list A list of background images. The ith list entry should be the background for camera id i.
    # @param mask_list A list of masks. The ith list entry should be the mask for camera id i.
    # @param threshold  The cut off brightness value for converting from a gray scale to binary image.
    # @param erode_iterations The number of times to erode the binary image. This is used to eliminate small detected blobs. More iterations removes larger blobs.
    # @param min_blob_size The minimum expected blob size in square pixels for a robot.
    # @param max_blob_size The maximum expected blob size in square pixels for a robot.
    # @param storage An OpenCV storage container created with cv.CreateMemStorage.
    # @param image_width Width of the image to be tracked in pixels.
    # @param image_height Height of the image to be tracked in pixels.
    # @param scale The converstion scale between pixels and meters in meters/pixel.
    def __init__(self, background_list, mask_list, binary_threshold, erode_iterations, min_blob_size, max_blob_size, storage, image_width, image_height, scale):
        DcslVisionTracker.__init__(self, background_list, mask_list, binary_threshold, erode_iterations, min_blob_size, max_blob_size, storage, image_width, image_height)
        self.scale = scale
        
        ## @var scale
        # The conversion scale between pixels and meters in meters/pixel.


    ## Applies coordinate transform from image reference frame into real reference frame to image_poses and returns sensed_poses.
    #
    # Return sensed_poses is a list of DcslPose objects in a right hand coordinate system in meters and radians centered at the center of the image with x up in the image frame and y to the right. Theta measured CCW from x axis.
    # @param image_poses a list of poses in image (pixel) coordinates. Top left corner is the origin with the y axis down and the x axis right. Theta measured CCW from x axis.
    # @param estimated_poses (List of DcslPose objects) not required for miabot tracker.
    # @param camera_id (int) not required for miabot tracker.    
    def image_to_world(self, image_poses, estimated_poses = None, camera_id = None):
        sensed_poses = []
        for pose in image_poses:
            sensed_pose = DcslPose()
            if pose.position[0] is not None:
                sensed_x = (pose.position[0]-0.5*self.image_width)*self.scale
                sensed_y = -(pose.position[1]-0.5*self.image_height)*self.scale
                sensed_pose.set_position((sensed_x,sensed_y,0))
                sensed_pose.set_quaternion((None,None,pose.quaternion[2],None))
                if pose.detected:
                    sensed_pose.set_detected(True)
            sensed_poses.append(sensed_pose)
        return sensed_poses

    ## Applies coordinate transform from world frame to image frame on world_poses and returns image_poses.
    #
    # Return image_poses is a list of DcslPose objects in image (pixel) coordinates. Top left corner is the origin with the y axis down and the x axis right. Theta is measured CCW from x axis.
    # @param world_poses a list of poses in the world reference frame (meters and radians).
    # @param camera_id (int) not required for miabot tracker.
    def world_to_image(self, world_poses, camera_id = None):
        image_poses = []
        for pose in world_poses:
            image_pose = DcslPose()
            if pose.position[0] is not None:
                image_x = pose.position[0]*self.scale + 0.5*self.image_width
                image_y = -1.0*pose.position[1]*self.scale - 0.5*self.image_height
                image_pose.set_position((image_x, image_y, None))
                image_pose.set_quaternion((None, None, pose.quaternion[2], None))
                if pose.detected:
                    image_pose.set_detected(True)
            image_poses.append(image_pose)
        return image_poses


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
            if blob_size > self.min_blob_size and blob_size < self.max_blob_size:
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
                # Append found pose to image_poses list
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

class DcslBelugaTracker(DcslVisionTracker):

    ## Class initializer. Loads arguments into object variables.
    #
    # @param background_list A list of background images. The ith list entry should be the background for camera id i.
    # @param mask_list A list of masks. The ith list entry should be the mask for camera id i.
    # @param threshold  The cut off brightness value for converting from a gray scale to binary image.
    # @param erode_iterations The number of times to erode the binary image. This is used to eliminate small detected blobs. More iterations removes larger blobs.
    # @param min_blob_size The minimum expected blob size in square pixels for a robot.
    # @param max_blob_size The maximum expected blob size in square pixels for a robot.
    # @param storage An OpenCV storage container created with cv.CreateMemStorage.
    # @param image_width Width of the image to be tracked in pixels.
    # @param image_height Height of the image to be tracked in pixels.
    # @param scale The converstion scale between pixels and normalized coordinates in 1/pixel.
    # @param translation_offset_list A list of tuples (o_x, o_y) where the ith tuple is the translation offset for camera i in meters.
    # @param camera_height Height of camera above the water surface in meters.
    # @param refraction_ratio (float) The ratio of the index of refraction of air to the index of refraction of water.
    def __init__(self, background_list, mask_list, binary_threshold, erode_iterations, min_blob_size, max_blob_size, storage, image_width, image_height, scale, translation_offset_list, camera_height, refraction_ratio):
        DcslVisionTracker.__init__(self, background_list, mask_list, binary_threshold, erode_iterations, min_blob_size, max_blob_size, storage, image_width, image_height)
        self.scale = scale
        self.translation = translation_offset_list
        self.camera_height = camera_height
        self.refraction_ratio = refraction_ratio
        
        ## @var scale
        # The conversion scale between pixels and meters in meters/pixel.

        ## @var translation
        # A list of tuples (o_x, o_y) where the ith tuple is the translation offset for camera i in meters.
        
        ## @var camera_height
        # Height of the camera above the water surface in meters.

        ## @var refraction_ratio
        # The ratio of the index of refraction of air to the index of refraction of water.
     
    
    ##
    #
    #
    def convert_to_grayscale(self, image):
        grayscale_image = cv.CreateMat(image.height, image.width, cv.CV_8UC1)
        cv.CvtColor(image, grayscale_image, cv.CV_BGR2GRAY)
        return grayscale_image
   

    ## Applies coordinate transform from image reference frame into real reference frame to image_poses and returns sensed_poses.
    #
    # Return sensed_poses is a list of DcslPose objects in a right hand coordinate system in meters and radians centered at the center of the image with x up in the image frame and y to the right. Theta measured CCW from x axis.
    # @param image_poses a list of poses in image (pixel) coordinates. Top left corner is the origin with the y axis down and the x axis right. Theta measured CCW from x axis.
    # @param estimated_poses (List of DcslPose objects) not required for miabot tracker.
    # @param camera_id (int) not required for miabot tracker.    
    def image_to_world(self, image_poses, estimated_poses, camera_id):
        world_poses = []
        R_x = self.translation[camera_id][0]
        R_y = self.translation[camera_id][1]
        R_z = self.translation[camera_id][2]
        alpha = self.scale
        for idx, pose in enumerate(image_poses):
            world_pose = DcslPose()
            if pose.position[0] is not None:
                z_world = estimated_poses[idx].position_z()
                x_image = pose.position_x()
                y_image = pose.position_y()
                z_camera = -1*(z_world-R_z)
                beta = self._beta(z_camera)
                x_camera = 1/alpha*(x_image - 0.5*self.image_width)*z_camera*beta
                y_camera = 1/alpha*(y_image - 0.5*self.image_height)*z_camera*beta
                x_world = x_camera + R_x
                y_world = -y_camera + R_y
                world_pose.set_position((x_world, y_world, z_world))
                world_pose.set_quaternion((None, None, pose.quaternion_z(), None))
            world_pose.set_detected(pose.detected)
            world_poses.append(world_pose)       
        return world_poses

    def _beta(self, z_camera):
        H = self.camera_height
        rir = self.refraction_ratio
        beta = pow(rir + (1 - rir)*H/z_camera, 0.5)
        return beta


    ## Applies coordinate transform from world frame to image frame on world_poses and returns image_poses.
    #
    # Return image_poses is a list of DcslPose objects in image (pixel) coordinates. Top left corner is the origin with the y axis down and the x axis right. Theta is measured CCW from x axis.
    # @param world_poses a list of poses in the world reference frame (meters and radians).
    # @param camera_id (int) not required for miabot tracker.
    def world_to_image(self, world_poses, camera_id):
        image_poses = []
        R_x = self.translation[camera_id][0]
        R_y = self.translation[camera_id][1]
        R_z = self.translation[camera_id][2]
        alpha = self.scale
        for pose in world_poses:
            image_pose = DcslPose()
            if pose.position[0] is not None:
                x_world = pose.position_x()
                y_world = pose.position_y()
                z_world = pose.position_z()
                x_camera = x_world - R_x
                y_camera = -1*(y_world - R_y)
                z_camera = -1*(z_world - R_z)
                beta = self._beta(z_camera)
                x_camera_prime = x_camera * 1/beta
                y_camera_prime = y_camera * 1/beta
                x_image = alpha * x_camera_prime/z_camera + self.image_width*0.5
                y_image = alpha * y_camera_prime/z_camera + self.image_height*0.5
                image_pose.set_position((x_image, y_image, z_world))
                image_pose.set_quaternion((None, None, pose.quaternion_z(), None))
            image_pose.set_detected(pose.detected)
            image_poses.append(image_pose)
        return image_poses

    def _f(self, theta1, x, y, z):
        f = self.camera_height*m.tan(theta1) - z*m.tan(self.refraction_ratio*theta1/pow(1-pow(self.refraction_ratio*theta1,2), 0.5))-(pow(x,2)+pow(y,2))
        return f
    def _delf(self, theta1, x, y, z):
        delf = self.camera_height*pow(m.cos(theta1),-2) - self.refraction_ratio*z*pow(m.cos(self.refraction_ratio*theta1/pow(1-pow(self.refraction_ratio*theta1,2), 0.5)), -2)/pow(1-pow(self.refraction_ratio*theta1, 2), 1.5)
        return delf


    ## Takes the contours found in an image and returns the positions and headings of the blobs in image frame coordinates
    #
    # Returns image_poses (list of DcslPose objects) in the coordinate frame of the image of all contours in unknown order
    # @param contours (CvSeq of contours) contours of the blobs found in an image 
    def get_image_poses(self, contours):
        image_poses = [] # List to store measured poses
        cr = contours #Copy contours to not change original variable
        if len(cr) > 0:
            while cr is not None:
                # Find area of blob and reject those not the size of a robot
                blob_size = cv.ContourArea(cr)
                if blob_size > self.min_blob_size and blob_size < self.max_blob_size:
                    moments = cv.Moments(cr, False)
                    '''
                    # Fit ellipse to blob
                    PointArray2D32f = cv.CreateMat(1, len(cv), cv.CV_32FC2)
                    for (i, (x,y)) in enumerate(c):
                    PointArray2D32f[0,i] = (x,y)
                    ellipse = cv.FitEllipse2(PointArray2D32f)
                    # Find center and centroid of blob
                    center = (ellipse[0][0], ellipse[0][1])
                    '''
                    # Find center of blob
                    box = cv.MinAreaRect2(cr, cv.CreateMemStorage())
                    ellipse = box
                    center = (box[0][0], box[0][1])
                    centroid = (cv.GetSpatialMoment(moments,1,0)/cv.GetSpatialMoment(moments,0,0),cv.GetSpatialMoment(moments,0,1)/cv.GetSpatialMoment(moments,0,0))
                    # Estimate heading by drawing line from centroid to center and finding heading of line
                    theta_estimate = -m.atan2(centroid[1]-center[1],centroid[0]-center[0])
                    # Theta is found using the minimum area box from above
                    theta_box = -m.pi/180.0*ellipse[2] #Angle in the first quadrant of the best fit ellipse with the x-axis
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
                        # Append found pose to image_poses list
                    pose = DcslPose()
                    pose.set_position((center[0],center[1],0))
                    pose.set_quaternion((0,0,theta,0))
                    image_poses.append(pose)
                    del ellipse, box                
                #Cycle to next contour
                cr=cr.h_next()
            del cr 
        return image_poses
            
        

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
