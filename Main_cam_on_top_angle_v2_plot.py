import cv2,math,time,keyboard,csv,cmath,os
import pyrealsense2 as rs
import numpy as np
from Scripts.Kinematic import inverse_kinematic,forwad_kinematic,forwad_kinematic_v2
from Scripts.Camera import Inital_color,Object_3D_recontruction,Camera_top_to_qlobal_coords
from Scripts.miscellaneous import _map,setp_to_list,list_to_setp
from Scripts.UR10 import initial_communiation
from Scripts.trajectory import asym_trajectory,inital_parameters_traj

detected = False
show_cam = False
flip_cam = False

#Config IntelRealsens
lower_color, upper_color = Inital_color("yellowbox")

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 848,480, rs.format.bgr8, 60)
# config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)
log_x = []
log_y = []
log_angle_x = []
log_end_effector_x = []
log_end_effector_y = []
log_angle_y = []
log_time = []
path = r"C:\Users\mateusz.jedynak\OneDrive - NTNU\Programmering\Python\Prosjekt\Bachelor\Source\Bachelor\Data\angle_pos_PID_endeffect_angle_v2"

#===============================End of functions=================================================================================
def Angle(UR_10_x,UR_10_y,object_x,object_y,l):
    return cmath.asin((object_x-UR_10_x)/l).real,cmath.asin((object_y-UR_10_y)/l).real
def trap_integrate(dt,a,v,q,a_prev):
    #inital values:
    q_0 = q
    v_0 = v
    #intrgrate acceralation:
    v = ((a+a_prev)/2) * dt + v_0
    #integrate velocity:
    q = ((v + v_0)/2) * dt + q_0
    return q,v,a
def main():
    # Client has a few methods to get proxy to UA nodes that should always be in address space such as Root or Objects
    setp,con,watchdog,Init_pose = initial_communiation('169.254.182.10', 30004,500)


    Kp_y, Kd_y, Ki_y = 0.5, 0.01,0.006
    Kp_x, Kd_x, Ki_x = 0.5, 0.01,0.006
    v_0_x,v_2_x,v_0_y,v_2_y,t_0,t_1,t_f = 0,0,0,0,0,1.5,0.75
    prev_error_x, prev_error_y,reference_point_x,reference_point_y = 0,0,0,0
    eintegral_x,eintegral_y = 0,0
    distance = 1.2  #1.48                                     # distance to object/payload
    pf = True
    pf_1 = True
    pf_2 = True
    angle_regulation = True
    prev_angle_x,prev_angle_y = 0,0
    prev_velocity_x,prev_velocity_y = 0,0
    running = False
    start_time = time.perf_counter()
    start_time_pos = 0
    start_time_traj = 0
    end_time = 0
    watchdog.input_int_register_0 = 2
    con.send(watchdog)  # sending mode == 2
    state = con.receive()
    pos_x,pos_y = 0,0
    prev_a_x,prev_a_y=0,0
    end_time_pos = 0
    t = 0
    traj_time = 10
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

        global_coordinates = Camera_top_to_qlobal_coords(coordinates_meters, forward_kinematic, angle_base)
        

        #Delta time 
        dt = time.perf_counter() - start_time
        start_time = time.perf_counter()
        angle = Angle(forward_kinematic_rope[0],forward_kinematic_rope[1],global_coordinates[0],global_coordinates[1],distance)
        cv2.putText(image, f"Angle : {angle[0]}", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255),2)
        if global_coordinates[0] > 0.1 and reference_point_x == 0:
            print("Start Regulation")
            start_time_log = time.time()
            reference_point_x,reference_point_y = 0.10, -0.7977911479915556
        elif reference_point_x == 0:
            con.send(watchdog)  # sending mode == 2
            state = con.receive()
        if reference_point_x != 0 or reference_point_y !=0:
            if abs(angle[0]) > 0.04 and abs(angle[1]) > 0.04 or end_time_pos < 2:
                angle_regulation = True
                error_x = (angle[0])
                dedt = (error_x-prev_error_x)/dt #Derivative
                prev_error_x = error_x
                eintegral_x = eintegral_x + error_x*dt #Integralk

                p_out_x = Kp_x*error_x + Kd_x*dedt + Ki_x*eintegral_x

                #PID y
                error_y = (angle[1])
                dedt = (error_y-prev_error_y)/dt #Derivative
                prev_error_y = error_y
                eintegral_y = eintegral_y + error_y*dt #Integralk

                p_out_y = Kp_y*error_y + Kd_y*dedt + Ki_y*eintegral_y
                if abs(angle[0]) < 0.04 and abs(angle[1]) < 0.04:
                    if pf_1:
                        print("pf start")
                        start_time_pos = time.time()
                        pf_1 = False
                    if start_time_pos != 0:
                        end_time_pos = time.time() - start_time_pos
                else:
                    end_time_pos = 0
            if abs(angle[0]) < 0.04 and abs(angle[1]) < 0.04 and end_time_pos > 2:
                angle_regulation = False
                if pf_2:
                    start_pose = forward_kinematic_rope
                    start_time_traj = time.time()
                    pf_2 = False

                if start_time_traj != 0:
                    t = time.time() - start_time_traj
                # print(t)
                end_time_pos = 15
                traj_time = max(abs(start_pose[0]),abs(start_pose[1]+0.885))*20
                parameters_to_trajectory_x = inital_parameters_traj(start_pose[0],     0,0,0,     0,      traj_time,    traj_time/2)
                parameters_to_trajectory_y = inital_parameters_traj(start_pose[1],-0.885,0,0,     0,      traj_time,    traj_time/2)
                                # Trajectory for y
                # print(traj_time)
                q_y, dq_y, ddq_y = asym_trajectory(t,parameters_to_trajectory_y)
                Init_pose[1] = q_y

                #Trajectory for x
                q_x, dq_x, ddq_x = asym_trajectory(t,parameters_to_trajectory_x)
                Init_pose[0] = q_x
                # print(Init_pose[0],Init_pose[1],t)
            pos_x = global_coordinates[0] - (distance*math.sin(p_out_x)) 
            pos_y = global_coordinates[1] - (distance*math.sin(p_out_y))
            cv2.putText(image, f"end_time_pos : {end_time_pos}", (100,150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255),2)
            # cv2.putText(image, f"pos_y : {pos_y}", (100,200), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255),2)

            state = con.receive()
            if state.runtime_state > 1 and detected:# and abs(angle[0]) > 0.03 :
                if watchdog.input_int_register_0 != 2:
                    watchdog.input_int_register_0 = 2
                    con.send(watchdog)  # sending mode == 4
                # Inverse Kinematic
                try:
                    if angle_regulation:
                        q1, q2, q3 = inverse_kinematic(pos_x,pos_y,0.750)
                        q6 = q2 +q3 +math.pi/2
                        send_to_ur = [q1,q2,q3,-1.570796327,-3.141592654,q6]
                    else:
                        q1, q2, q3 = inverse_kinematic(Init_pose[0], Init_pose[1], Init_pose[2])
                        q6 = q2 +q3 +math.pi/2
                        send_to_ur = [q1,q2,q3,-1.570796327,-3.141592654,q6]
                        # Init_pose[1],Init_pose[0] = p_out_y,p_out_x
                    cv2.putText(image, f"send_to_ur: {send_to_ur}", (100,400), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255),2)

                    list_to_setp(setp, send_to_ur)
                    con.send(setp)  # sending new8 pose
                except ValueError as info:
                    print(info)
                # print(end_time_pos)
                #Update values
                v_0_y,v_0_x = state.actual_TCP_speed[1],state.actual_TCP_speed[0]
                v_2_y,v_2_x = v_0_y,v_0_x
                # cv2.putText(image, f"Angle acceleration: {x_acceleration}", (100,150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255),2)
                endtime = time.time()- start_time_log
                log_time.append(endtime)
                log_x.append(global_coordinates[0])
                log_y.append(global_coordinates[1])
                if angle_regulation:
                    log_end_effector_x.append(pos_x)
                    log_end_effector_y.append(pos_y)
                else:
                    log_end_effector_x.append(Init_pose[0])
                    log_end_effector_y.append(Init_pose[1])
                log_angle_x.append(angle[0])
                log_angle_y.append(angle[1])
            else:
                if watchdog.input_int_register_0 != 4:
                    watchdog.input_int_register_0 = 4
                    con.send(watchdog)  # sending mode == 4
            
            if keyboard.is_pressed("esc") or t > traj_time:  # Break loop with ESC-key
                info_csv_1 = [f"Posisjonering til lasten er 62,5 grade fra UR10, Y: -140 X: -55"]
                info_csv_2 = [f"Kp_x:{Kp_x}, Kp_y:{Kp_y}, Kd_x:{Kd_x}, Kd_y:{Kd_y}, Ki_x: {Ki_x}, Ki_x: {Ki_y}, referance point {0.0276}, {-0.8846}"]
                header = ["Time","X","Y","AngleX","AngleY","End_effector_X","End_effector_Y"]
                with open(path + '\X-Y-ulike_PID_{}.csv'.format(str(len(os.listdir(path)))), 'w',newline="") as f:
                    # create the csv writer
                    writer = csv.writer(f)
                    writer.writerow(info_csv_1)
                    writer.writerow(info_csv_2)
                    writer.writerow(header)
                    for i in range(len(log_time)):
                        writer.writerow([log_time[i],log_x[i],log_y[i],log_angle_x[i],log_angle_y[i],log_end_effector_x[i],log_end_effector_y[i]])
                print("Ferdig")
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
    try:
        main()
    except Exception as e:
        print(e)
        pipeline.stop()

