"""
Code written based on servoj example from: https://github.com/davizinho5/RTDE_control_example
"""
import cv2,math,torch,sys,asyncio,logging,time
import pyrealsense2 as rs
import numpy as np
from Scripts.Kinematic import inverse_kinematic
from Scripts.Camera import ObjectDetection,Inital_color
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
    while 1:
        frames = pipeline.wait_for_frames() #

        aligned_frames =  align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        aligned_color_frame = aligned_frames.get_color_frame()

        image = np.asanyarray(aligned_color_frame.get_data())
        # depth = np.asanyarray(depth_frame.get_data())

        #Object detection

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # Creates a mask
        mask = cv2.inRange(hsv, lower_color, upper_color)
        # Enlarge the mask
        kernel = np.ones((5, 5), np.uint8)
        dilation = cv2.dilate(mask, kernel)
        # Finding the contours
        contours, hierarchy = cv2.findContours(dilation, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            box = max(contours, key=cv2.contourArea)
            if cv2.contourArea(box) > 1500:
                cv2.drawContours(image, box, -1, (0, 255, 0), 2)
                M = cv2.moments(box)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                x_cord = center[0]-width/2
                y_cord = -1*(center[1]-height/2)

                #Distance
                cv2.circle(image, center, 3, (0, 0, 255), -1)                
                distnace_to_send = np.array([depth_frame.get_distance(center[0], center[1])
                                            ,depth_frame.get_distance(center[0]+5, center[1]+5)
                                            ,depth_frame.get_distance(center[0]-5, center[1]-5)
                                            ,depth_frame.get_distance(center[0]+5, center[1]-5)
                                            ,depth_frame.get_distance(center[0]-5, center[1]+5)])
                distnace_to_send = np.mean(distnace_to_send[distnace_to_send != 0])

                print(distnace_to_send)
                cv2.putText(image, "centroid", (center[0] + 10, center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255),1)
                cv2.putText(image, "(" + str(x_cord) + "," + str(y_cord) + ")", (center[0] + 10, center[1] + 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
        cv2.imshow("Result", image)
        # cv2.imshow("Mask", mask)
        # cv2.imshow("Depth", depth)
        if cv2.waitKey(1) == 27:  # Break loop with ESC-key
            break   
if __name__ == '__main__':
    main()