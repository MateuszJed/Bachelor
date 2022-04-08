"""
Code written based on servoj example from: https://github.com/davizinho5/RTDE_control_example
"""
import cv2,math,torch,sys,asyncio,logging,time
import pyrealsense2 as rs
import numpy as np
from Scripts.Kinematic import inverse_kinematic
from Scripts.Camera_v2 import ObjectDetection,Inital_color
from Scripts.miscellaneous import _map,setp_to_list,list_to_setp
from Scripts.UR10 import initial_communiation
from Scripts.trajectory import asym_trajectory,inital_parameters_traj
import matplotlib.pyplot as plt

lower_color, upper_color = Inital_color("yellowbox")

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
    # Client has a few methods to get proxy to UA nodes that should always be in address space such as Root or Objects
    middle_point = 1.2368000507354737
    while 1:


        
        frames = pipeline.wait_for_frames() #
        
        aligned_frames =  align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        aligned_color_frame = aligned_frames.get_color_frame()

        image = np.asanyarray(aligned_color_frame.get_data())
        depth = np.asanyarray(depth_frame.get_data())

        #Object detection
        x_send, y_send,x_m,distance,image, mask, depth, detected = ObjectDetection(image, depth_frame,depth, lower_color,
                                                                                 upper_color, height, width,middle_point, flip_cam)
        # print(x_pos,y_pos)
        cv2.imshow("Result", image)
        # print(distance)
        # cv2.imshow("Mask", mask)
        # cv2.imshow("Depth", depth)
        if cv2.waitKey(1) == 27:  # Break loop with ESC-key
            pipeline.stop()
            break   
if __name__ == '__main__':
    main()