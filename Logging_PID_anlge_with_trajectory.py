import cv2,math,time,keyboard,csv,cmath,os,pyautogui
import pyrealsense2 as rs
import numpy as np
from Scripts.Kinematic import inverse_kinematic,forwad_kinematic,forwad_kinematic_v2
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
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
# config.enable_stream(rs.stream.color, 848,480, rs.format.bgr8, 60)
pipeline.start(config)

log_x = []
log_y = []
log_angle_x = []
log_end_effector_x = []
log_end_effector_y = []
log_angle_y = []
log_time = []
path = r"C:\Users\mateusz.jedynak\OneDrive - NTNU\Programmering\Python\Prosjekt\Bachelor\Source\Bachelor\Data\testing"
def main():
    # Client has a few methods to get proxy to UA nodes that should always be in address space such as Root or Objects
    setp,con,watchdog,Init_pose = initial_communiation('169.254.182.10', 30004,500,"pid")
    pyautogui.hotkey('ctrl', 'r')  # ctrl-c to copy

    Kp_y, Kd_y, Ki_y = 0.5, 0.01,0.006
    Kp_x, Kd_x, Ki_x = 0.5, 0.01,0.006
    distance = 1.2                                      # distance to object/payload
    REFERENCE_POINT_X = 0
    REFERENCE_POINT_Y = -0.885
    REFERENCE_POINT_Z = 0.750
    SCALE_FACTOR_FOR_TRJECETORY = 20
    prev_error_x, prev_error_y,reference_point_x,reference_point_y = 0,0,0,0
    eintegral_x,eintegral_y = 0,0
    prev_angle_x,prev_angle_y = 0,0
    prev_velocity_x,prev_velocity_y = 0,0

    pf = True
    pf_1 = True
    pf_2 = True
    angle_regulation = True
    running = False
    regulation_start = False

    start_time = time.perf_counter()
    start_time_pos = 0
    start_time_traj = 0
    end_time = 0

    watchdog.input_int_register_0 = 2
    con.send(watchdog)  # sending mode == 2
    state = con.receive()
    pos_x,pos_y = 0,0
    end_time_pos = 0
    t = 0
    traj_time = 999
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
        # cv2.putText(image, f"Angle in x-direction: {angle[0]}", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255),2)
        # cv2.putText(image, f"Angle in y-direction: {angle[1]}", (100,150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255),2)
        if global_coordinates[0] > 0.1 and not(regulation_start):
            print("Start Regulation")
            start_time_log = time.time()
            regulation_start = True
        if regulation_start:
            if abs(angle[0]) > 0.04 and abs(angle[1]) > 0.04 or end_time_pos < 2:
                angle_regulation = True

                #PID x
                p_out_x,error_x,eintegral_x = PID(Kp_x,Kd_x,Ki_x,angle[0],0,prev_error_x,eintegral_x,dt,1)
                prev_error_x = error_x
                #PID y
                p_out_y,error_y,eintegral_y = PID(Kp_y,Kd_y,Ki_y,angle[1],0,prev_error_y,eintegral_y,dt,1)
                prev_error_y = error_y

                if abs(angle[0]) < 0.04 and abs(angle[1]) < 0.04:
                    if pf_1:
                        print("Start trajectory planner")
                        start_time_pos = time.time()
                        pf_1 = False
                    if start_time_pos != 0:
                        end_time_pos = time.time() - start_time_pos
                else:
                    end_time_pos = 0

                #From angle to the postion of end effector
                pos_x = global_coordinates[0] - (distance*math.sin(p_out_x)) 
                pos_y = global_coordinates[1] - (distance*math.sin(p_out_y))
                cv2.putText(image, f"end_time_pos : {end_time_pos}", (100,150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255),2)

            if abs(angle[0]) < 0.04 and abs(angle[1]) < 0.04 and end_time_pos > 2:
                angle_regulation = False
                if pf_2:
                    start_pose = forward_kinematic_rope
                    start_time_traj = time.time()
                    pf_2 = False

                if start_time_traj != 0:
                    t = time.time() - start_time_traj
                traj_time = max(abs(start_pose[0]),abs(start_pose[1]+abs(REFERENCE_POINT_Y)))*SCALE_FACTOR_FOR_TRJECETORY 
                parameters_to_trajectory_x = inital_parameters_traj(start_pose[0],REFERENCE_POINT_X,0,0,     0,      traj_time,    traj_time/2)
                parameters_to_trajectory_y = inital_parameters_traj(start_pose[1],REFERENCE_POINT_Y,0,0,     0,      traj_time,    traj_time/2)
                # Trajectory for y
                q_y= asym_trajectory(t,parameters_to_trajectory_y)
                Init_pose[1] = q_y

                #Trajectory for x
                q_x= asym_trajectory(t,parameters_to_trajectory_x)
                Init_pose[0] = q_x

            state = con.receive()
            if state.runtime_state > 1 and detected:
                if watchdog.input_int_register_0 != 2:
                    watchdog.input_int_register_0 = 2
                    con.send(watchdog)  # sending mode == 4
                # Inverse Kinematic
                try:
                    if angle_regulation:
                        q1, q2, q3 = inverse_kinematic(pos_x,pos_y,REFERENCE_POINT_Z)
                        q6 = q2 +q3 +math.pi/2
                        send_to_ur = [q1,q2,q3,-1.570796327,-3.141592654,q6]
                    else:
                        q1, q2, q3 = inverse_kinematic(Init_pose[0], Init_pose[1], Init_pose[2])
                        q6 = q2 +q3 +math.pi/2
                        send_to_ur = [q1,q2,q3,-1.570796327,-3.141592654,q6]

                    list_to_setp(setp, send_to_ur)
                    con.send(setp)  # sending new8 pose
                except ValueError as info:
                    print(info)

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
                pyautogui.hotkey('ctrl', 't')  # ctrl-v to paste

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
            regulation_start = True
            start_time_log = time.time()
            pf = False
            print(f"Start Ragulate")          
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

