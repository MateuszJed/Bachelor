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
from Scripts.trajectory import asym_trajectory, log_traj, plot_traj ,inital_parameters_traj
import matplotlib.pyplot as plt

lower_color, upper_color = Inital_color("yellowbox")

flip_cam = False
intel_cam = True
detected = False
Kp = 0.5
kd = 0.1
#If IntelSense are not connecet switch to pc camera
try:
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    pipeline.start(config)
except RuntimeError as info:
    if str(info) == "No device connected":
        cap = cv2.VideoCapture(0)
        succes, image = cap.read()
        height, width, channels = image.shape
        intel_cam = False
if intel_cam:
    # Get information from IntelSens camera
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()

    image = np.asanyarray(color_frame.get_data())
    height = image.shape[0]
    width = image.shape[1]

def main():
    # Client has a few methods to get proxy to UA nodes that should always be in address space such as Root or Objects
    setp,con,watchdog,Init_pose = initial_communiation('169.254.182.10', 30004,500)
    v_0,v_2,t_0,t_1,t_f = 0    ,0,     0,      1.5,    0.75

    reference_point = 0
    start_time = time.time()
    watchdog.input_int_register_0 = 2
    con.send(watchdog)  # sending mode == 2
    state = con.receive()
    prev_error = 0
    while 1:
        if intel_cam:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            # Convert images to numpy arrays
            image = np.asanyarray(color_frame.get_data())
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        else:
            #Read pc camera
            success, image = cap.read()
        #Object detection
        x_send, y_send, mask,image,detected = ObjectDetection(image,lower_color, upper_color,height,width,flip_cam)
        #Constrain values from camera 
        x_send = _map(x_send,-width/2,width/2,-1,1)*-1
        #y_send = _map(y_send,-height/2,height/2,100,500)
        cv2.imshow("Result", image)
        if reference_point != 0:
            #PID
            error = (reference_point-x_send)
            P_out = Kp*error+kd*(error-prev_error)
            
            # Trajectory 

            T = inital_parameters_traj(Init_pose[0],P_out,0    ,0,     0,      1.5,    0.75)

            t = time.time() - start_time
            state = con.receive()
            if state.runtime_state > 1 and detected:
                if watchdog.input_int_register_0 != 2:
                    watchdog.input_int_register_0 = 2
                    con.send(watchdog)  # sending mode == 4
                q, dq, ddq = asym_trajectory(t)
                # logging trajectory
                Init_pose[0] = q
                q1, q2, q3 = inverse_kinematic(Init_pose[0], Init_pose[1], Init_pose[2])
                send_to_ur = [q1,q2,q3,-1.570796327,-3.141592654,1.570796327]

                list_to_setp(setp, send_to_ur)
                con.send(setp)  # sending new pose
            else:
                if watchdog.input_int_register_0 != 4:
                    watchdog.input_int_register_0 = 4
                    con.send(watchdog)  # sending mode == 4

            

            v_0 = state.actual_TCP_speed[0]
            v_2 = v_0
            Init_pose[0] = P_out
            start_time = time.time()
            prev_error = error
            if cv2.waitKey(1) == 27:  # Break loop with ESC-key
                state = con.receive()
                # ====================mode 3===================
                watchdog.input_int_register_0 = 3
                con.send(watchdog)
                con.send_pause()
                con.disconnect()
                break   
        if cv2.waitKey(1) == ord("k"):
            reference_point = Init_pose[0]
            print("Reference point its ready")
if __name__ == '__main__':
    main()