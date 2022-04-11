from turtle import distance
import cv2,math,torch,sys,asyncio,logging,time,keyboard,csv,os
import pyrealsense2 as rs
import numpy as np
from Scripts.Kinematic import inverse_kinematic
from Scripts.Camera_v2 import ObjectDetection,Inital_color
from Scripts.miscellaneous import _map,setp_to_list,list_to_setp
from Scripts.UR10 import initial_communiation
from Scripts.trajectory import asym_trajectory,inital_parameters_traj

lower_color, upper_color = Inital_color("yellowbox")
flip_cam = False
detected = False
Controll = True
run = True

path = r"C:\Users\mateusz.jedynak\OneDrive - NTNU\Programmering\Python\Prosjekt\Bachelor\Source\Bachelor\Data\X-Y-meter_PID"


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

log_x = []
log_distance = []
log_time = []
def Camera_to_global_coords(x,y,z):
    global_coords= np.array(np.array([[1,0,0,0],[0,0,1,-2.184],[0,-1,0,-0.662],[0,0,0,1]]))@np.array([[x],[y],[z],[1]])
    return global_coords[0][0],global_coords[1][0],global_coords[2][0]
def main():
    # Client has a few methods to get proxy to UA nodes that should always be in address space such as Root or Objects
    setp,con,watchdog,Init_pose = initial_communiation('169.254.182.10', 30004,500)
    Kp_y, Kd_y, Ki_y = 0.5, 0.003,0.007
    Kp_x, Kd_x, Ki_x = 0.5, 0.003,0.007
    v_0_x,v_2_x,v_0_y,v_2_y,t_0,t_1,t_f = 0,0,0,0,0,1.5,0.75
    prev_error_x, prev_error_y,reference_point_x,reference_point_y = 0,0,0,0
    eintegral_x,eintegral_y = 0,0

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
        camera_coordinates, detected = ObjectDetection(image,color_frame, depth_frame, lower_color, upper_color, flip_cam)
        x,y,z = Camera_to_global_coords(camera_coordinates[0],camera_coordinates[1],camera_coordinates[2])
        xlogging = x

        #Delta time 
        t = time.time() - start_time
        start_time = time.time()
        if x > 0.1 and reference_point_x == 0:
            print("Start Regulation")
            start_time_log = time.time()
            reference_point_x,reference_point_y = 0.01249997429549694,-0.9055999364852907

        if reference_point_x != 0 or reference_point_y !=0:
            #PID Y
            error_y = (reference_point_y-y)*-1
            dedt = (error_y-prev_error_y)/t #Derivative
            eintegral_y = eintegral_y + error_y*t #Integral

            P_out_yy = Kp_y*error_y + Kd_y*dedt + Ki_y*eintegral_y
            P_out_y = P_out_yy + reference_point_y

            #PID X
            error_x = (reference_point_x-x)*-1
            dedt = (error_x-prev_error_x)/t #Derivative
            eintegral_x = eintegral_x + error_x*t #Integral

            P_out_xx = Kp_x*error_x + Kd_x*dedt + Ki_x*eintegral_x
            P_out_x = P_out_xx + reference_point_x

            # Trajectory 
            parameters_to_trajectory_y = inital_parameters_traj(Init_pose[1],P_out_y,v_0_y,v_2_y,     0,      0.1,    0.05)
            parameters_to_trajectory_x = inital_parameters_traj(Init_pose[0],P_out_x,v_0_x,v_2_x,     0,      1.5,    0.75)
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
                    q6 = q2 +q3 +math.pi/2
                    send_to_ur = [q1,q2,q3,-1.570796327,-3.141592654,q6]

                    list_to_setp(setp, send_to_ur)
                    con.send(setp)  # sending new8 pose
                except ValueError as info:
                    print(info)
                log_time.append(endtime)
                log_x.append(x)
                log_distance.append(y)
                if endtime > 15:
                    running = True
            else:
                if watchdog.input_int_register_0 != 4:
                    watchdog.input_int_register_0 = 4
                    con.send(watchdog)  # sending mode == 4
            

            v_0_y,v_0_x = state.actual_TCP_speed[1],state.actual_TCP_speed[0]
            v_2_y,v_2_x = v_0_y,v_0_x
            Init_pose[1],Init_pose[0] = P_out_y,P_out_x

            if keyboard.is_pressed("esc") or running:  # Break loop with ESC-key
                info_csv_1 = [f"Posisjonering til lasten er 62,5 grade fra UR10, Y: -140 X: -55"]
                info_csv_2 = [f"Kp_x:{Kp_x}, Kp_y:{Kp_y}, Kd_x:{Kd_x}, Kd_y:{Kd_y}, Ki_x: {Ki_x}, Ki_x: {Ki_y}"]
                header = ["Time","X","Y"]
                with open(r'C:\Users\mateusz.jedynak\OneDrive - NTNU\Programmering\Python\Prosjekt\Bachelor\Source\Bachelor\Data\X-Y-meter_PID\X-Y-meter_PID_{}.csv'.format(str(len(os.listdir(path)))), 'w',newline="") as f:
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
            if keyboard.is_pressed("q"):  # Break loop with ESC-key
                Kp_y = Kp_y + 0.1
            if keyboard.is_pressed("a"):  # Break loop with ESC-key
                Kp_y = Kp_y - 0.1
            if keyboard.is_pressed("w"):  # Break loop with ESC-key
                Kd_y = Kd_y + 0.001
            if keyboard.is_pressed("s"):  # Break loop with ESC-key
                Kd_y = Kd_y - 0.001
            if keyboard.is_pressed("e"):  # Break loop with ESC-key
                Ki_y = Ki_y + 0.001
            if keyboard.is_pressed("d"):  # Break loop with ESC-key
                Ki_y = Ki_y - 0.001
        if keyboard.is_pressed("k"):
            reference_point_y = y
            reference_point_x = x
            start_time_log = time.time()
            Controll = True
            print("Reference point its ready")            
if __name__ == '__main__':
    main()
