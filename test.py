"""
Code written based on servoj example from: https://github.com/davizinho5/RTDE_control_example
"""
import cv2,math,sys,asyncio,logging,time
import pyrealsense2 as rs
import numpy as np
from Scripts.Kinematic import inverse_kinematic
from Scripts.Camera import Object_3D_recontruction,Inital_color,Camera_to_global_coords
from Scripts.miscellaneous import _map,setp_to_list,list_to_setp
from Scripts.UR10 import initial_communiation
from Scripts.trajectory import asym_trajectory,inital_parameters_traj
import matplotlib.pyplot as plt

lower_color, upper_color = Inital_color("yellowBox")

flip_cam = False
detected = False

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)
align_to = rs.stream.depth
align = rs.align(align_to)

# Get information from IntelSens camera
frames = pipeline.wait_for_frames()
color_frame = frames.get_color_frame()
image = np.asanyarray(color_frame.get_data())
height = image.shape[0]
width = image.shape[1]

def main():
    while 1:
        frames = pipeline.wait_for_frames() #
        
        aligned_frames =  align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        aligned_color_frame = aligned_frames.get_color_frame()

        image = np.asanyarray(aligned_color_frame.get_data())
        depth = np.asanyarray(depth_frame.get_data())
        #Object detection
        x_pix, y_pix , camera_x_meters, camera_y_meters, camera_z_meters= Object_3D_recontruction(image,color_frame ,depth_frame,depth,
                                                                      lower_color, upper_color, flip_cam)
        #print(f'x_pixel: {x_pix}   y_pixel: {y_pix}    X: {camera_x_meters}    Y: {camera_y_meters}    Z: {camera_z_meters}')

        Base_X,Base_Y, Base_Z = Camera_to_global_coords(camera_x_meters, camera_y_meters, camera_z_meters)
        print(f'Global coords:   X: {Base_X*1000}   Y: {Base_Y*1000}    Z: {Base_Z*1000}')

        cv2.imshow("Result", image)
        # cv2.imshow("Mask", mask)
        #cv2.imshow("Depth", depth)
        if cv2.waitKey(1) == 27:  # Break loop with ESC-key
            pipeline.stop()
            break   
if _name_ == '_main_':
    main()