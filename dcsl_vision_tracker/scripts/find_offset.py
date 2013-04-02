#!/usr/bin/env python

import dcsl_vision_tracker_API as d
import sys

def main(x_image, y_image):
    
    z_world = 0
    
    camera_height = 3.14
    water_depth = 2.38

    scale = pow(1.45/3.05*1.0/204.0, -1)

    refraction_ratio = 1.0/1.333

    image_width = 640
    image_height = 480

    R_z = camera_height+water_depth

    image_pose = d.DcslPose()
    image_pose.set_position((x_image, y_image, 0))
    image_pose.set_quaternion((0,0,0,0))
    image_poses = [image_pose]
    
    estimated_pose = d.DcslPose()
    estimated_pose.set_position((0,0,0))
    estimated_pose.set_quaternion((0,0,0,0))
    estimated_poses = [estimated_pose]
        

    tracker = d.DcslBelugaTracker([],[None],None,None,None,None,None,image_width, image_height,scale,[(0,0,R_z)],camera_height, refraction_ratio)
    world_poses = tracker.image_to_world(image_poses, estimated_poses,0)
    R_x = -1*world_poses[0].position_x()
    R_y = -1*world_poses[0].position_y()
    print("R_x: " + str(R_x))
    print("R_y: " + str(R_y))
    print("R_z: " + str(R_z))

if __name__ == "__main__":
    args = sys.argv
    x_image = float(args[1])
    y_image = float(args[2])
    main(x_image, y_image)
