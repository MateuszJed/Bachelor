from tkinter.tix import Tree
import cv2,math,time,keyboard
import pyrealsense2 as rs
import numpy as np
from Scripts.Kinematic import inverse_kinematic,forwad_kinematic
from Scripts.Camera import Inital_color,Object_3D_recontruction,Camera_top_to_qlobal_coords
from Scripts.miscellaneous import _map,setp_to_list,list_to_setp,PID
from Scripts.UR10 import initial_communiation
from Scripts.trajectory import asym_trajectory,inital_parameters_traj

show_cam = True
flip_cam = False

#Config IntelRealsens
lower_color, upper_color = Inital_color("yellowbox")

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 848,480, rs.format.bgr8, 60)
# config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)

def main():
    # Client has a few methods to get proxy to UA nodes that should always be in address space such as Root or Objects
    setp,con,watchdog,Init_pose = initial_communiation('169.254.182.10', 30004,500)
    Kp_y, Kd_y, Ki_y = 0.5, 0.006,0.006
    Kp_x, Kd_x, Ki_x = 0.5, 0.005,0.006
    v_0_x,v_2_x,v_0_y,v_2_y,t_0,t_1,t_f = 0,0,0,0,0,1.5,0.75
    prev_error_x, prev_error_y,reference_point_x,reference_point_y = 0,0,0,0
    eintegral_x,eintegral_y = 0,0
    distance = 1.2                                       # distance to object/payload

    detected = False
    running = False
    pf = True
    start_time = time.perf_counter()
    watchdog.input_int_register_0 = 2
    con.send(watchdog)  # sending mode == 2
    state = con.receive()

    while state.runtime_state > 1:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        image = np.asanyarray(color_frame.get_data())
        #Object detection
        coordinates_pix, coordinates_meters,detected = Object_3D_recontruction( image,          color_frame,
                                                                                lower_color,    upper_color, 
                                                                                flip_cam,       distance)
        forward_kinematic = state.actual_TCP_pose[:3]
        angle_base = state.actual_q[0]

        global_coordinates = Camera_top_to_qlobal_coords(coordinates_meters, forward_kinematic, angle_base)
        #Delta time 
        dt = time.perf_counter() - start_time
        start_time = time.perf_counter()
        if reference_point_x != 0 or reference_point_y !=0:

            #PID x
            P_out_x,error_x,eintegral_x = PID(Kp_x,Kd_x,Ki_x,reference_point_x,global_coordinates[0],prev_error_x,eintegral_x,dt,-1)
            prev_error_x = error_x

            #PID y
            P_out_y,error_y,eintegral_y = PID(Kp_y,Kd_y,Ki_y,reference_point_y,global_coordinates[1],prev_error_y,eintegral_y,dt,-1)
            prev_error_y = error_y

            # Trajectory 
            parameters_to_trajectory_y = inital_parameters_traj(Init_pose[1],P_out_y,v_0_y,v_2_y,     0,      0.1,    0.05)
            parameters_to_trajectory_x = inital_parameters_traj(Init_pose[0],P_out_x,v_0_x,v_2_x,     0,      1.5,    0.75)

            state = con.receive()
            if state.runtime_state > 1 and detected:
                if watchdog.input_int_register_0 != 2:
                    watchdog.input_int_register_0 = 2
                    con.send(watchdog)  # sending mode == 4

                #Trajectory for y
                Init_pose[1] = asym_trajectory(dt,parameters_to_trajectory_y)
                #Trajectory for x
                Init_pose[0] = asym_trajectory(dt,parameters_to_trajectory_x)

                #Inverse Kinematic
                try: 
                    q1, q2, q3 = inverse_kinematic(Init_pose[0], Init_pose[1], Init_pose[2])
                    q6 = q2 +q3 +math.pi/2
                    send_to_ur = [q1,q2,q3,-1.570796327,-3.141592654,q6]
                    list_to_setp(setp, send_to_ur)
                    con.send(setp)  # sending new8 pose
                except ValueError as info:
                    print(info)

                #Update values
                v_0_y,v_0_x = state.actual_TCP_speed[1],state.actual_TCP_speed[0]
                v_2_y,v_2_x = v_0_y,v_0_x
                Init_pose[1],Init_pose[0] = P_out_y,P_out_x
            else:
                if watchdog.input_int_register_0 != 4:
                    watchdog.input_int_register_0 = 4
                    con.send(watchdog)  # sending mode == 4
            
            if keyboard.is_pressed("esc"):  # Break loop with ESC-key
                state = con.receive()
                # ====================mode 3===========k========
                watchdog.input_int_register_0 = 3
                con.send(watchdog)
                con.send_pause()
                con.disconnect()
                pipeline.stop()
                break
        else:
            #Sending mode == 2 if no referece point
            con.send(watchdog)  # sending mode == 2
            state = con.receive()
        if keyboard.is_pressed("k") and pf:
            reference_point_x = state.actual_TCP_pose[0]
            reference_point_y = state.actual_TCP_pose[1]
            print(f"Reference point its ready: {reference_point_x}, {reference_point_y}")
            pf = False        
        if show_cam:
            cv2.imshow("Result", image)
            if cv2.waitKey(1):  # Break loop with ESC-key
                pass
        
if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
        pipeline.stop()