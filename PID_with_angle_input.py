import cv2,math,time,keyboard,cmath
import pyrealsense2 as rs
import numpy as np
from Scripts.Kinematic import inverse_kinematic,forwad_kinematic_v2
from Scripts.Camera import Inital_color,Object_3D_recontruction,Camera_top_to_qlobal_coords
from Scripts.miscellaneous import _map,setp_to_list,list_to_setp,Angle,PID
from Scripts.UR10 import initial_communiation
from Scripts.trajectory import asym_trajectory,inital_parameters_traj

detected = False
show_cam = True
flip_cam = False

#Config IntelRealsens
lower_color, upper_color = Inital_color("yellowbox")

pipeline = rs.pipeline()
config = rs.config()
# config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.color, 848,480, rs.format.bgr8, 60)
pipeline.start(config)

def main():
    # Client has a few methods to get proxy to UA nodes that should always be in address space such as Root or Objects
    setp,con,watchdog,Init_pose = initial_communiation('169.254.182.10', 30004,500,"pid")

    REFERENCE_POINT_X = 0
    REFERENCE_POINT_Y = -0.885
    REFERENCE_POINT_Z = 0.750

    Kp_y, Kd_y, Ki_y = 0.5, 0.01,0.006
    Kp_x, Kd_x, Ki_x = 0.5, 0.01,0.006
    v_0_x,v_2_x,v_0_y,v_2_y,t_0,t_1,t_f = 0,0,0,0,0,1.5,0.75
    prev_error_x, prev_error_y,reference_point_x,reference_point_y = 0,0,0,0
    eintegral_x,eintegral_y = 0,0
    distance = 1.2                           # distance to object/payload
    pf = True
    prev_angle_x,prev_angle_y = 0,0
    prev_velocity_x,prev_velocity_y = 0,0

    running = False
    start_time = time.perf_counter()
    watchdog.input_int_register_0 = 2
    con.send(watchdog)  # sending mode == 2
    state = con.receive()
    pos_x,pos_y = 0,0
    prev_a_x,prev_a_y=0,0

    while state.runtime_state > 1:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        image = np.asanyarray(color_frame.get_data())
        #Object detection
        coordinates_pix, coordinates_meters,detected = Object_3D_recontruction( image,          color_frame,
                                                                                lower_color,    upper_color, 
                                                                                flip_cam,       distance)
        forward_kinematic = state.actual_TCP_pose[:3]
        forward_kinematic_rope = forwad_kinematic_v2(state.actual_q[0],state.actual_q[1],state.actual_q[2])
        angle_base = state.actual_q[0]

        global_coordinates = Camera_top_to_qlobal_coords(coordinates_meters, forward_kinematic, angle_base)
        

        #Delta time 
        dt = time.perf_counter() - start_time
        start_time = time.perf_counter()
        angle = Angle(forward_kinematic_rope[0],forward_kinematic_rope[1],global_coordinates[0],global_coordinates[1],distance)
        if reference_point_x != 0 or reference_point_y !=0:
            #PID x
            p_out_x,error_x,eintegral_x = PID(Kp_x,Kd_x,Ki_x,angle[0],0,prev_error_x,eintegral_x,dt,1)
            prev_error_x = error_x
            #PID y
            p_out_y,error_y,eintegral_y = PID(Kp_y,Kd_y,Ki_y,angle[1],0,prev_error_y,eintegral_y,dt,1)
            prev_error_y = error_y

            state = con.receive()
            if state.runtime_state > 1 and detected:
                if watchdog.input_int_register_0 != 2:
                    watchdog.input_int_register_0 = 2
                    con.send(watchdog)  # sending mode == 4
                # Inverse Kinematic
                try: 
                    q1, q2, q3 = inverse_kinematic(p_out_x+REFERENCE_POINT_X,p_out_y+REFERENCE_POINT_Y,REFERENCE_POINT_Z)
                    q6 = q2 +q3 +math.pi/2
                    send_to_ur = [q1,q2,q3,-1.570796327,-3.141592654,q6]

                    list_to_setp(setp, send_to_ur)
                    con.send(setp)  # sending new8 pose
                except ValueError as info:
                    print(info)
            else:
                if watchdog.input_int_register_0 != 4:
                    watchdog.input_int_register_0 = 4
                    con.send(watchdog)  # sending mode == 4
            
            if keyboard.is_pressed("esc"):  # Break loop with ESC-key
                state = con.receive()
                # ====================mode 3===================
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
            reference_point_x = global_coordinates[0]
            reference_point_y = global_coordinates[1]
            pf = False
            print(f"Reference point its ready: {reference_point_x}, {reference_point_y}")          
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

