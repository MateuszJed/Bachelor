import cv2,math,time,keyboard,csv,cmath,os,pyautogui
import pyrealsense2 as rs
import numpy as np
from Scripts.Kinematic import inverse_kinematic,forwad_kinematic,forwad_kinematic_v2
from Scripts.Camera import Inital_color,Object_3D_recontruction,Camera_top_to_qlobal_coords
from Scripts.miscellaneous import _map,setp_to_list,list_to_setp,Angle
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
    # pyautogui.hotkey('ctrl', 'r')  # ctrl-c to copy


    Kp_y, Kd_y, Ki_y = 0.5, 0.01,0.006
    Kp_x, Kd_x, Ki_x = 0.5, 0.01,0.006
    v_0_x,v_2_x,v_0_y,v_2_y,t_0,t_1,t_f = 0,0,0,0,0,1.5,0.75
    prev_error_x, prev_error_y,reference_point_x,reference_point_y = 0,0,0,0
    eintegral_x,eintegral_y = 0,0
    distance = 1.48  #1.48                                     # distance to object/payload
    pf = True
    prev_angle_x,prev_angle_y = 0,0
    prev_velocity_x,prev_velocity_y = 0,0
    running = False
    start_time = time.time()
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
        
        if global_coordinates[0] > 0.1 and reference_point_x == 0:
            print("Start Regulation")
            start_time_log = time.time()
            reference_point_x,reference_point_y = 0.10, -0.7977911479915556
        elif reference_point_x == 0:
            con.send(watchdog)  # sending mode == 2
            state = con.receive()
        if reference_point_x != 0 or reference_point_y !=0:
            #PID X
            error_x = (angle[0])
            dedt = (error_x-prev_error_x)/dt #Derivative
            prev_error_x = error_x
            eintegral_x = eintegral_x + error_x*dt #Integralk

            pos_x = Kp_x*error_x + Kd_x*dedt + Ki_x*eintegral_x
            #PID y
            error_y = (angle[1])
            dedt = (error_y-prev_error_y)/dt #Derivative
            prev_error_y = error_y
            eintegral_y = eintegral_y + error_y*dt #Integralk

            pos_y = Kp_y*error_y + Kd_y*dedt + Ki_y*eintegral_y

            # cv2.putText(image, f"Angle : {angle[0]*180/math.pi,angle[1]*180/math.pi}", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255),2)
            # cv2.putText(image, f"global coordinates: {global_coordinates}", (100,150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255),2)


            state = con.receive()
            if state.runtime_state > 1 and detected:
                if watchdog.input_int_register_0 != 2:
                    watchdog.input_int_register_0 = 2
                    con.send(watchdog)  # sending mode == 4
                # Inverse Kinematic
                try: 
                    q1, q2, q3 = inverse_kinematic(pos_x+0.0276,pos_y-0.8846,0.750)
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
                # cv2.putText(image, f"Angle acceleration: {x_acceleration}", (100,150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255),2)
                endtime = time.time()- start_time_log
                log_time.append(endtime)
                log_x.append(global_coordinates[0])
                log_y.append(global_coordinates[1])
                log_angle_x.append(angle[0])
                log_angle_y.append(angle[1])
                log_end_effector_x.append(pos_x+0.0276)
                log_end_effector_y.append(pos_y-0.8846)
                #End regulation after x-sec
                # if endtime > 10:
                #     running = True
            else:
                if watchdog.input_int_register_0 != 4:
                    watchdog.input_int_register_0 = 4
                    con.send(watchdog)  # sending mode == 4
            
            if keyboard.is_pressed("esc") or running:  # Break loop with ESC-key
                # pyautogui.hotkey('ctrl', 't')  # ctrl-c to copy

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
    try:
        main()
    except Exception as e:
        print(e)
        pipeline.stop()

