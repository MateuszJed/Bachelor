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
Kp_y, Kd_y = 0.5, 0.1
Kp_x, Kd_x = 0.5, 0.1
flip = 1
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
    setp,con,watchdog,Init_pose = initial_communiation('169.254.182.10', 30004,500)

    v_0_x,v_2_x,v_0_y,v_2_y,t_0,t_1,t_f = 0,0,0,0,0,1.5,0.75
    prev_error_x, prev_error_y,reference_point_x,reference_point_y = 0,0,0,0
    running = False

    start_time = time.time()
    watchdog.input_int_register_0 = 2
    con.send(watchdog)  # sending mode == 2
    state = con.receive()
    while 1:
        frames = pipeline.wait_for_frames()
        aligned_frames =  align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        aligned_color_frame = aligned_frames.get_color_frame()

        image = np.asanyarray(aligned_color_frame.get_data())
        depth = np.asanyarray(depth_frame.get_data())
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        #Object detection
        x_send, y_send,distance,image, mask, depth, detected = ObjectDetection(image, depth_frame,depth, lower_color,
                                                                                 upper_color, height, width, flip_cam)
        xlogging = x_send
        # cv2.imshow("Result",image)
        # print(x_send)
        if x_send > -10 and reference_point_x == 0:
            flip = 1
            start_time_log = time.time()
            reference_point_y = 1.3874000787734986
            reference_point_x = 0.012723869889305114

        if reference_point_x != 0 and reference_point_y !=0:
            #PID Y
            error_y = (reference_point_y-distance)*flip
            P_out_y = Kp_y*error_y+Kd_y*(error_y-prev_error_y)
            P_out_y = _map(P_out_y,Constrain_y[0],Constrain_y[1],Constrain_y[2],Constrain_y[3])

            #Constrain values from camera in x-axis
            x_send = _map(x_send,Constrain_x[0],Constrain_x[1],Constrain_x[2],Constrain_x[3])*-1
            #PID X
            error_x = (reference_point_x-x_send)*flip
            P_out_x = Kp_x*error_x+Kd_x*(error_x-prev_error_x)
            # Trajectory 
            parameters_to_trajectory_y = inital_parameters_traj(Init_pose[1],P_out_y,v_0_y,v_2_y,     0,      1.5,    0.75)
            parameters_to_trajectory_x = inital_parameters_traj(Init_pose[0],P_out_x,v_0_x,v_2_x,     0,      1.5,    0.75)
            
            t = time.time() - start_time
            state = con.receive()
            if state.runtime_state > 1 and detected:
                if watchdog.input_int_register_0 != 2:
                    watchdog.input_int_register_0 = 2
                    con.send(watchdog)  # sending mode == 4

                endtime = time.time()- start_time_log

                #Trajectory for y
                q_y, dq_y, ddq_y = asym_trajectory(t,parameters_to_trajectory_y)
                Init_pose[1] = q_y

                #Trajectory for x
                q_x, dq_x, ddq_x = asym_trajectory(t,parameters_to_trajectory_x)
                Init_pose[0] = q_x

                #Inverse Kinematic
                try: 
                    q1, q2, q3 = inverse_kinematic(Init_pose[0], Init_pose[1], Init_pose[2])
                    send_to_ur = [q1,q2,q3,-1.570796327,-3.141592654,1.570796327]

                    list_to_setp(setp, send_to_ur)
                    con.send(setp)  # sending new8 pose
                except ValueError as info:
                    print(info)
                log_time.append(endtime)
                log_x.append(xlogging)
                log_distance.append(distance-reference_point_y)
                if endtime > 15:
                    running = True
                    flip = -1
                    if endtime > 25:
                        reference_point_x = 0

            else:
                if watchdog.input_int_register_0 != 4:
                    watchdog.input_int_register_0 = 4
                    con.send(watchdog)  # sending mode == 4
            

            
            v_0_y,v_0_x = state.actual_TCP_speed[1],state.actual_TCP_speed[0]
            v_2_y,v_2_x = v_0_y,v_0_x
            Init_pose[1],Init_pose[0] = P_out_y,P_out_x
            start_time = time.time()
            if keyboard.is_pressed("esc"):# or running:  # Break loop with ESC-key
                info_csv_1 = [f"Constrain_x: {Constrain_x}, Constrain_y: {Constrain_y}, Posisjonering til lasten er 62,5 grade fra UR10, Y: -140 X: -55"]
                info_csv_2 = [f"Kp_x:{Kp_x}, Kp_y:{Kp_y}, Kd_x:{Kd_x}, Kd_y:{Kd_y}"]
                header = ["Time","X","Y"]
                with open(r'C:\Users\mateusz.jedynak\OneDrive - NTNU\Programmering\Python\Prosjekt\Bachelor\Source\Bachelor\Data\X-Y-retning-pix-meter_simply_PID\X-Y-retning-pix-meter_simply_PID_{}.csv'.format(str(len(os.listdir(path)))), 'w',newline="") as f:
                    # create the csv writer
                    writer = csv.writer(f)
                    writer.writerow(info_csv_1)
                    writer.writerow(info_csv_2)
                    writer.writerow(header)
                    for i in range(len(log_time)):
                        writer.writerow([log_time[i],log_x[i],log_distance[i]])
                print("Ferdig")   
                state = con.receive()
                # ====================mode 3===================
                watchdog.input_int_register_0 = 3
                con.send(watchdog)
                con.send_pause()
                con.disconnect()
        
        if keyboard.is_pressed("k"):
            reference_point_y = 1.3874000787734986
            reference_point_x = 0.012723869889305114
            start_time_log = time.time()
            Controll = True
            print("Reference point its ready")            
if __name__ == '__main__':
    main()
