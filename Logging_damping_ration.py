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
path = r"C:\Users\mateusz.jedynak\OneDrive - NTNU\Programmering\Python\Prosjekt\Bachelor\Source\Bachelor\Data\Dampeing_ratio"

def main():
    # Client has a few methods to get proxy to UA nodes that should always be in address space such as Root or Objects
    setp,con,watchdog,Init_pose = initial_communiation('169.254.182.10', 30004,500,"pid")
    reference_point_x,reference_point_y = 0,0
    distance = 1.48  #1.48                                     # distance to object/payload

    running = False
    start_time = time.time()
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
            state = con.receive()
            if state.runtime_state > 1 and detected:
                if watchdog.input_int_register_0 != 2:
                    watchdog.input_int_register_0 = 2
                    con.send(watchdog)  # sending mode == 4
                # Inverse Kinematic
               
                # cv2.putText(image, f"Angle acceleration: {x_acceleration}", (100,150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255),2)
                endtime = time.time()- start_time_log
                log_time.append(endtime)
                log_x.append(global_coordinates[0])
                log_y.append(global_coordinates[1])
                log_angle_x.append(angle[0])
                log_angle_y.append(angle[1])
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
                info_csv_2 = [f"Posisjonering til lasten er 62,5 grade fra UR10, Y: -140 X: -55"]
                header = ["Time","X","Y","AngleX","AngleY"]
                with open(path + '\Damp_ratio-{}.csv'.format(str(len(os.listdir(path)))), 'w',newline="") as f:
                    # create the csv writer
                    writer = csv.writer(f)
                    writer.writerow(info_csv_1)
                    writer.writerow(info_csv_2)
                    writer.writerow(header)
                    for i in range(len(log_time)):
                        writer.writerow([log_time[i],log_x[i],log_y[i],log_angle_x[i],log_angle_y[i]])
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

