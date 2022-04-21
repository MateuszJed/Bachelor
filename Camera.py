import cv2,math,torch,sys,asyncio,logging,time,keyboard,csv,os
import pyrealsense2 as rs
import numpy as np
from Scripts.Kinematic import inverse_kinematic
from Scripts.Camera import ObjectDetection,Inital_color
from Scripts.miscellaneous import _map,setp_to_list,list_to_setp
from Scripts.UR10 import initial_communiation
from Scripts.trajectory import asym_trajectory,inital_parameters_traj

lower_color, upper_color = Inital_color("yellowbox")

flip_cam = False
detected = False
Controll = True
run = True
Kp_y, Kd_y,Ki_y = 1, 0.3,3
Kp_x, Kd_x,Ki_x = 0.5, 0.1,1

path = r"C:\Users\mateusz.jedynak\OneDrive - NTNU\Programmering\Python\Prosjekt\Bachelor\Source\Bachelor\Data\X-Y-retning-pix-meter_simply_PID"


#Config IntelRealsens
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
Constrain_y = [-1.5,1.5,-0.3,-1.4]
Constrain_x = [-width/2,width/2,-0.8,0.8]

log_x = []
log_distance = []
log_time = []

def main():
    # Client has a few methods to get proxy to UA nodes that should always be in address space such as Root or Objects
    v_0_x,v_2_x,v_0_y,v_2_y,t_0,t_1,t_f = 0,0,0,0,0,1.5,0.75
    prev_error_x, prev_error_y,reference_point_x,reference_point_y = 0,0,0,0
    running = False
    while 1:
        frames = pipeline.wait_for_frames()
        aligned_frames =  align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        aligned_color_frame = aligned_frames.get_color_frame()
        start_time = time.time()

        image = np.asanyarray(aligned_color_frame.get_data())
        depth = np.asanyarray(depth_frame.get_data())
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #Object detection
        x_send, y_send,distance,image, mask, depth, detected = ObjectDetection(image, depth_frame,depth, lower_color,
                                                                                 upper_color, height, width, flip_cam)
        dt = time.time() - start_time
        cv2.putText(image, "FPS:" + str(30.0), (1100,100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 75, 75),2)
        cv2.imshow("Result",image)

        if cv2.waitKey(1) == 27:  # Break loop with ESC-key
            pipeline.stop()
            break   
if __name__ == '__main__':
    main()
