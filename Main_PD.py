import cv2,math,torch,sys,asyncio,logging,time
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
run = True
Kp_y, Kd_y = 0.5, 0
Kp_x, Kd_x = 0.5, 0.1

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

def main():
    # Client has a few methods to get proxy to UA nodes that should always be in address space such as Root or Objects
    setp,con,watchdog,Init_pose = initial_communiation('169.254.182.10', 30004,500)

    v_0_x,v_2_x,v_0_y,v_2_y,t_0,t_1,t_f = 0,0,0,0,0,1.5,0.75
    prev_error_x, prev_error_y,reference_point_x,reference_point_y = 0,0,0,0

    start_time = time.time()
    watchdog.input_int_register_0 = 2
    con.send(watchdog)  # sending mode == 2
    state = con.receive()
    while 1:
        frames = pipeline.wait_for_frames() #
        aligned_frames =  align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        aligned_color_frame = aligned_frames.get_color_frame()

        image = np.asanyarray(aligned_color_frame.get_data())
        depth = np.asanyarray(depth_frame.get_data())
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        #Object detection
        x_send, y_send,distance,image, mask, depth, detected = ObjectDetection(image, depth_frame,depth, lower_color,
                                                                                 upper_color, height, width, flip_cam)

       

        cv2.imshow("Result", image)
        if reference_point_x != 0 and reference_point_y !=0:
            #PID Y
            error_y = (reference_point_y-distance)
            P_out_y = Kp_y*error_y+Kd_y*(error_y-prev_error_y)-0.68
            P_out_y = _map(P_out_y,-1.5,1.5,-0.3,-1.4)

            #Constrain values from camera in x-axis
            x_send = _map(x_send,-width/2,width/2,-0.8,0.8)*-1
            #PID X
            error_x = (reference_point_x-x_send)
            P_out_x = Kp_x*error_x#+Kd_x*(error_x-prev_error_x)
            print(P_out_y,P_out_x)
            # Trajectory 
            parameters_to_trajectory_y = inital_parameters_traj(Init_pose[1],P_out_y,v_0_y,v_2_y,     0,      1.5,    0.75)
            parameters_to_trajectory_x = inital_parameters_traj(Init_pose[0],P_out_x,v_0_x,v_2_x,     0,      1.5,    0.75)

            t = time.time() - start_time
            state = con.receive()
            if state.runtime_state > 1 and detected:
                if watchdog.input_int_register_0 != 2:
                    watchdog.input_int_register_0 = 2
                    con.send(watchdog)  # sending mode == 4

                #Trajectory for y
                q_y, dq_y, ddq_y = asym_trajectory(t,parameters_to_trajectory_y)
                Init_pose[1] = q_y

                #Trajectory for x
                q_x, dq_x, ddq_x = asym_trajectory(t,parameters_to_trajectory_x)
                Init_pose[0] = q_x

                #Inverse Kinematic
                q1, q2, q3 = inverse_kinematic(Init_pose[0], Init_pose[1], Init_pose[2])
                send_to_ur = [q1,q2,q3,-1.570796327,-3.141592654,1.570796327]

                list_to_setp(setp, send_to_ur)
                con.send(setp)  # sending new pose
            else:
                if watchdog.input_int_register_0 != 4:
                    watchdog.input_int_register_0 = 4
                    con.send(watchdog)  # sending mode == 4


            v_0_y,v_0_x = state.actual_TCP_speed[1],state.actual_TCP_speed[0]
            v_2_y,v_2_x = v_0_y,v_0_x
            Init_pose[1],Init_pose[0] = P_out_y,P_out_x

            start_time = time.time()
            if cv2.waitKey(1) == 27:  # Break loop with ESC-key
                state = con.receive()
                # ====================mode 3===================
                watchdog.input_int_register_0 = 3
                con.send(watchdog)
                con.send_pause()
                con.disconnect()
        if cv2.waitKey(1) == ord("k"):
            reference_point_y = distance
            reference_point_x = Init_pose[0]
            print("Reference point its ready")
            
if __name__ == '__main__':
    main()
