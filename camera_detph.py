from turtle import distance
import cv2,math,torch,sys,asyncio,logging,time,keyboard,csv,os
import pyrealsense2 as rs
import numpy as np
from Scripts.Kinematic import inverse_kinematic
from Scripts.Camera_v2 import ObjectDetection,Inital_color
from Scripts.miscellaneous import _map,setp_to_list,list_to_setp
from Scripts.UR10 import initial_communiation
from Scripts.trajectory import asym_trajectory,inital_parameters_traj

lower_color, upper_color = Inital_color("redtop")
flip_cam = False
detected = False
Controll = True
run = True


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

path = r"C:\Users\mateusz.jedynak\OneDrive - NTNU\Programmering\Python\Prosjekt\Bachelor\Source\Bachelor\Data\messurments"

def main():
    # Client has a few methods to get proxy to UA nodes that should always be in address space such as Root or Objects
    x = []
    y = []
    running = True
    log = False
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
        cv2.imshow("Result", image)
        if log:
            x.append(camera_coordinates[0])
            y.append(camera_coordinates[1])
            # print(camera_coordinates[0],camera_coordinates[1])
            if len(x) > 1000:
                info_csv_1 = [f"Posisjone til Lasten i globale kordinater "]
                #info_csv_2 = [
                    #f"P1:{}, Kp_y:{Kp_y}, Kd_x:{Kd_x}, Kd_y:{Kd_y}, Ki_x: {Ki_x}, Ki_x: {Ki_y}, referance point {reference_point_x}, {reference_point_y}"]
                header = ["X", "Y"]
                with open(path + '\Campbotton_measur_cam_{}_{}_{}.csv'.format(1280,720,str(len(os.listdir(path)))), 'w', newline="") as f:
                    # create the csv writer
                    writer = csv.writer(f)
                    writer.writerow(info_csv_1)
                    writer.writerow(header)
                    for i in range(len(x)):
                        writer.writerow([x[i], y[i]])
                print("Finish")
                running = False
                pipeline.stop()
                break
        if cv2.waitKey(1) == ord("w") and running:
            log = True
            print("Log")
        if cv2.waitKey(1) == 27:
                pipeline.stop()
                break    

if __name__ == '__main__':
    main()
