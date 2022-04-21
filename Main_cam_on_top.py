from mimetypes import init
import cv2,math,time,keyboard,csv,cmath
import pyrealsense2 as rs
import numpy as np
from Scripts.Kinematic import inverse_kinematic,forwad_kinematic,forwad_kinematic_v2
from Scripts.Camera import Inital_color,Object_3D_recontruction,Camera_top_to_qlobal_coords
from Scripts.miscellaneous import _map,setp_to_list,list_to_setp
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
#===============================End of functions=================================================================================
def Angle(UR_10_x,UR_10_y,object_x,object_y,l):
    return cmath.asin((object_x-UR_10_x)/l).real,cmath.asin((object_y-UR_10_y)/l).real

def main():
    # Client has a few methods to get proxy to UA nodes that should always be in address space such as Root or Objects
    setp,con,watchdog,Init_pose = initial_communiation('169.254.182.10', 30004,500)

    #PID values
    # Kp_y, Kd_y, Ki_y = 0.5, 0,0
    # Kp_x, Kd_x, Ki_x = 0.5, 0,0
    Kp_y, Kd_y, Ki_y = 0.5, 0.006,0.006
    Kp_x, Kd_x, Ki_x = 0.5, 0.005,0.006
    v_0_x,v_2_x,v_0_y,v_2_y,t_0,t_1,t_f = 0,0,0,0,0,1.5,0.75
    prev_error_x, prev_error_y,reference_point_x,reference_point_y = 0,0,0,0
    eintegral_x,eintegral_y = 0,0
    distance = 1.48  #1.48                                     # distance to object/payload
    pf = True
    running = False
    start_time = time.time()
    watchdog.input_int_register_0 = 2
    con.send(watchdog)  # sending mode == 2
    state = con.receive()
    prev_angle_x,prev_angle_y = 0,0
    prev_velocity_x,prev_velocity_y = 0,0
    while 1:
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
        # print(angle_base,forward_kinematic)
        global_coordinates = Camera_top_to_qlobal_coords(coordinates_meters, forward_kinematic, angle_base) 

        #Delta time 
        dt = time.time() - start_time
        start_time = time.time()

        angle = Angle(forward_kinematic_rope[0],forward_kinematic_rope[1],global_coordinates[0],global_coordinates[1],distance)
        # print(f"bef int(): {angle}")
        angle = round(angle[0],3),round(angle[1],3)
        # print(f"after int(): {angle}")

        # # print(reference_point_x,reference_point_y,error_x,error_y,global_coordinates)
        cv2.putText(image, f"Angle : {angle[0]*180/math.pi,angle[1]*180/math.pi}", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255),2)
        # cv2.putText(image, f"Pos : {global_coordinates[0],global_coordinates[1]}", (100,150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255),2)
        

        print(1/dt)
        if reference_point_x != 0 or reference_point_y !=0:
            #PID X
            error_x = (reference_point_x-global_coordinates[0])*-1
            dedt = (error_x-prev_error_x)/dt #Derivative
            prev_error_x = error_x
            eintegral_x = eintegral_x + error_x*dt #Integralk

            P_out_x = reference_point_x + Kp_x*error_x + Kd_x*dedt + Ki_x*eintegral_x

            #PID Y
            error_y = (reference_point_y-global_coordinates[1])*-1
            dedt = (error_y-prev_error_y)/dt #Derivative
            prev_error_y = error_y
            eintegral_y = eintegral_y + error_y*dt #Integral

            P_out_y = reference_point_y+  Kp_y*error_y + Kd_y*dedt + Ki_y*eintegral_y

                    # Angular acceleration
            x_velocity = (P_out_x-prev_angle_x)/(dt)
            prev_angle_x = P_out_x

            x_acceleration = (x_velocity-prev_velocity_x)/dt
            prev_velocity_x = x_velocity

            # Angular acceleration
            y_velocity = (P_out_y-prev_angle_y)/(dt)
            prev_angle_y = P_out_y

            y_acceleration = (y_velocity-prev_velocity_y)/dt
            prev_velocity_y = y_velocity
            # print(error_x,error_y)
            # cv2.putText(image, f"reference_point_y: {reference_point_y}", (100,150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255),2)
            # cv2.putText(image, f"error_x: {error_x}", (100,200), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255),2)
            # # cv2.putText(image, f"error_y: {error_y}", (100,250), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255),2)
            cv2.putText(image, f"global coordinates: {global_coordinates}", (100,150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255),2)
            cv2.putText(image, f"X velocity: {x_velocity}", (100,200), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255),2)
            cv2.putText(image, f"Y velocity: {y_velocity}", (100,250), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255),2)
            cv2.putText(image, f"X acceleration: {x_acceleration}", (100,300), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255),2)
            cv2.putText(image, f"Y acceleration: {y_acceleration}", (100,350), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255),2)
            # cv2.putText(image, f"Actual TCP_POS: {state.actual_TCP_pose}", (100,350), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255),2)
            
            # Trajectory x 0.0677 y  0.0082 x -0.0577
            parameters_to_trajectory_x = inital_parameters_traj(Init_pose[0]-0.0474,P_out_x-0.0474,v_0_x,v_2_x,     0,      0.1,    0.05)
            parameters_to_trajectory_y = inital_parameters_traj(Init_pose[1]-0.0352,P_out_y-0.0352,v_0_y,v_2_y,     0,      0.1,    0.05)
            # parameters_to_trajectory_x = inital_parameters_traj(Init_pose[0],P_out_x,v_0_x,v_2_x,     0,      1.5,    0.75)
            state = con.receive()
            if state.runtime_state > 1 and detected:
                if watchdog.input_int_register_0 != 2:
                    watchdog.input_int_register_0 = 2
                    con.send(watchdog)  # sending mode == 4

                #Trajectory for y
                q_y, dq_y, ddq_y = asym_trajectory(dt,parameters_to_trajectory_y)
                Init_pose[1] = q_y

                #Trajectory for x
                q_x, dq_x, ddq_x = asym_trajectory(dt,parameters_to_trajectory_x)
                Init_pose[0] = q_x

                # Inverse Kinematic
                try: 
                    q1, q2, q3 = inverse_kinematic(Init_pose[0], Init_pose[1], Init_pose[2])
                    q6 = q2 +q3 +math.pi/2
                    send_to_ur = [q1,q2,q3,-1.570796327,-3.141592654,q6]
                    # cv2.putText(image, f"send_to_ur: {send_to_ur}", (100,400), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255),2)

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
                # ====================mode 3===================
                watchdog.input_int_register_0 = 3
                con.send(watchdog)
                con.send_pause()
                con.disconnect()
                pipeline.stop()
                break
            if keyboard.is_pressed("y"):
                reference_point_x = reference_point_x + 0.01
                reference_point_y = reference_point_y+ 0.01
        else:
            #Sending mode == 2 if no referece point
            con.send(watchdog)  # sending mode == 2
            state = con.receive()
        if keyboard.is_pressed("k") and pf:
            # reference_point_x = global_coordinates[0]
            # reference_point_y = global_coordinates[1]
            reference_point_x = 0.041353141344053074
            reference_point_y = -0.8189731696854206
            start_time_log = time.time()
            pf = False
            print(f"Reference point its ready: {reference_point_x}, {reference_point_y}")          
        if show_cam:
            cv2.imshow("Result", image)
            if cv2.waitKey(1):  # Break loop with ESC-key
                pass
if __name__ == '__main__':
    # try:
    main()
    # except Exception as e:
    #     print(e)
    #     pipeline.stop()

