import cv2,math,torch,sys,asyncio,logging,time
import pyrealsense2 as rs
import numpy as np
from Scripts.Kinematic import inverse_kinematic
from Scripts.Camera import ObjectDetection,Inital_color
from Scripts.miscellaneous import _map,setp_to_list,list_to_setp
from Scripts.UR10 import initial_communiation
from Scripts.trajectory import asym_trajectory, log_traj, plot_traj ,inital_parameters_traj

lower_color, upper_color = Inital_color("redshit")

flip_cam = True
intel_cam = True
detected = False
run = True

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
        print(height, width)
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

    start_time = time.time()
    watchdog.input_int_register_0 = 2
    con.send(watchdog)  # sending mode == 2
    state = con.receive()
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
        x_send = _map(x_send,-width/2,width/2,-0.7,0.7)
        #y_send = _map(y_send,-height/2,height/2,100,500)

        # Trajectory 

        T = inital_parameters_traj(Init_pose[0],x_send,0    ,0,     0,      1.5,    0.75)

        state = con.receive()
        t = time.time() - start_time
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

        cv2.imshow("Result", image)

        v_0 = state.actual_TCP_speed[0]
        v_2 = v_0
        Init_pose[0] = x_send
        start_time = time.time()
        if cv2.waitKey(1) == 27:  # Break loop with ESC-key
            state = con.receive()
            # ====================mode 3===================
            watchdog.input_int_register_0 = 3
            con.send(watchdog)
            con.send_pause()
            con.disconnect()

            break   
            
if __name__ == '__main__':
    main()
    
    # setp,con,watchdog,Initial_pose = initial_communiation('169.254.182.10', 30004,500)

    # watchdog.input_int_register_0 = 2
    # con.send(watchdog)  # sending mode == 2
    # Init_pose = [-0.012687318175246987, 0.6870381118043345, 0.7403751487516523, -1.2215629305198221, 1.199165039620324, -1.1935355139916901]
    # Final_pose = [0.5077415367274349, 0.8677513175760304, 0.620792500956789, -1.6518454739898978, 1.3587417129274773, -1.980812429926074]

    # # #   -------------------------Control loop --------------------
    # #insert inital parametes: (q_0,q_1,v_0,v_2,t_0,t_1,t_f)
    # T = inital_parameters_traj(Init_pose[0],Final_pose[0],0,0,0,1.5,0.75)

    # start_time = time.time()
    # while time.time() - start_time < T:
    #     state = con.receive()
    #     # print(state.actual_TCP_pose)
    #     t = time.time() - start_time
    #     if state.runtime_state > 1:
    #         # calculation of trajectory
    #         q, dq, ddq = asym_trajectory(t)
    #         # logging trajectory
    #         Init_pose[0] = q
    #         q1, q2, q3 = inverse_kinematic(Init_pose[0], Init_pose[1], Init_pose[2])
    #         send_to_ur = [q1,q2,q3,-1.570796327,-3.141592654,1.570796327]

    #         list_to_setp(setp, send_to_ur)
    #         con.send(setp)  # sending new pose

    # state = con.receive()

    # # ====================mode 3===================
    # watchdog.input_int_register_0 = 3
    # con.send(watchdog)

    # con.send_pause()
    # con.disconnect()
    