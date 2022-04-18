import cv2,cmath
import time,csv,os
import numpy as np
import pyrealsense2 as rs
import math
# from miscellaneous import Angle

def Camera_top_to_qlobal_coords(camera_coords, translation, q1): 
    global_coords = np.array([[-np.sin(q1), np.cos(q1), 0], [np.cos(q1), np.sin(q1), 0], [0, 0, -1]]) @ np.c_[np.array(camera_coords)] + np.c_[translation]
    return (global_coords[0][0], global_coords[1][0], global_coords[2][0])

def Inital_color(name_of_list):
    # """ Calibration center of body."""
    color_list = []
    with open(r"C:\Users\mateusz.jedynak\OneDrive - NTNU\Programmering\Python\Prosjekt\Bachelor\Source\Bachelor\Color_data_set\{}.txt".format(name_of_list), "r") as f:
        for line in f:
            x = line.split(",")
            for i in x:
                if i.isdigit():
                    color_list.append(int(i))

    return np.array(color_list[:3]), np.array(color_list[3:])

def ObjectDetection(image,color_frame, depth_frame, lower_color, upper_color, flip_cam,show_cam):
    # Convert RGB to HSV
    if flip_cam:
        image = cv2.flip(image,-1)
        depth = cv2.flip(depth,-1)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Creates a mask
    mask = cv2.inRange(hsv, lower_color, upper_color)
    # Enlarge the mask
    kernel = np.ones((5, 5), np.uint8)
    dilation = cv2.dilate(mask, kernel)
    # Finding the contours
    contours, hierarchy = cv2.findContours(dilation, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        box = max(contours, key=cv2.contourArea)
        if cv2.contourArea(box) > 1500:
            cv2.drawContours(image, box, -1, (0, 255, 0), 2)
            M = cv2.moments(box)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            distance = np.array([depth_frame.get_distance(center[0], center[1])
                                        ,depth_frame.get_distance(center[0]+5, center[1]+5)
                                        ,depth_frame.get_distance(center[0]-5, center[1]-5)
                                        ,depth_frame.get_distance(center[0]+5, center[1]-5)
                                        ,depth_frame.get_distance(center[0]-5, center[1]+5)])
            distance = np.mean(distance[distance != 0])+0.07 #offsett from wall to center of box

            intrin = color_frame.profile.as_video_stream_profile().intrinsics
            camera_coordinates_3D= np.array([rs.rs2_deproject_pixel_to_point(intrin, [center[0], center[1]], distance),
                                            rs.rs2_deproject_pixel_to_point(intrin, [center[0] + 5, center[1] + 5], distance),
                                            rs.rs2_deproject_pixel_to_point(intrin, [center[0] - 5, center[1] - 5], distance),
                                            rs.rs2_deproject_pixel_to_point(intrin, [center[0] + 5, center[1] - 5], distance),
                                            rs.rs2_deproject_pixel_to_point(intrin, [center[0] - 5, center[1] + 5], distance)])


            camera_x_meters = np.mean(camera_coordinates_3D[:,0])
            camera_y_meters = np.mean(camera_coordinates_3D[:, 1])
            camera_z_meters = np.mean(camera_coordinates_3D[:, 2][camera_coordinates_3D[:, 2] != 0])
            x,y,z = Camera_to_global_coords(camera_x_meters,camera_y_meters,camera_z_meters)
            if show_cam:
                cv2.circle(image, center, 3, (0, 0, 255), -1)
                cv2.putText(image, "centroid", (center[0] + 10, center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255),1)
                cv2.putText(image, "(" + str(round(x,3)) + str(round(y,3)) + ")", (center[0] + 10, center[1] + 15),cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
            return [x,y,z],True
        else:
            return [0,0,0],False
    else:
            return [0,0,0],False

def Object_3D_recontruction(image, color_frame, lower_color, upper_color, flip_cam, distance):  
    # Convert RGB to HSV
    if flip_cam:
        image = cv2.flip(image, -1)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Creates a mask
    mask = cv2.inRange(hsv, lower_color, upper_color)
    # Enlarge the mask
    kernel = np.ones((5, 5), np.uint8)
    dilation = cv2.dilate(mask, kernel)
    # Finding the contours
    contours, hierarchy = cv2.findContours(dilation, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #object detection
    if len(contours) > 0:
        box = max(contours, key=cv2.contourArea)
        if cv2.contourArea(box) > 1500:
            cv2.drawContours(image, box, -1, (0, 255, 0), 2)
            M = cv2.moments(box)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            #get the intrinsic matrix
            color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
            #from pixel to meter: 
            camera_coordinates_3D = np.array(
                [rs.rs2_deproject_pixel_to_point(color_intrin, [center[0], center[1]], distance),
                 rs.rs2_deproject_pixel_to_point(color_intrin, [center[0] + 5, center[1] + 5], distance),
                 rs.rs2_deproject_pixel_to_point(color_intrin, [center[0] - 5, center[1] - 5], distance),
                 rs.rs2_deproject_pixel_to_point(color_intrin, [center[0] + 5, center[1] - 5], distance),
                 rs.rs2_deproject_pixel_to_point(color_intrin, [center[0] - 5, center[1] + 5], distance)])
            
            camera_x_meters = np.mean(camera_coordinates_3D[:, 0])
            camera_y_meters = np.mean(camera_coordinates_3D[:, 1])
            camera_z_meters = np.mean(camera_coordinates_3D[:, 2])

            cv2.circle(image, center, 3, (0, 0, 255), -1)
            return center, (camera_x_meters, camera_y_meters, camera_z_meters),True
        else:
            return (0, 0), (0, 0, 0),False
    else:
        return (0, 0), (0, 0, 0),False

# if __name__ == "__main__":
#     pipeline = rs.pipeline()
#     config = rs.config()
#     config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
#     config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
#     pipeline.start(config)
#     align_to = rs.stream.depth
#     align = rs.align(align_to)

#     frames = pipeline.wait_for_frames()
#     color_frame = frames.get_color_frame()
#     image = np.asanyarray(color_frame.get_data())


#     lower_color, upper_color = Inital_color("yellowbox")
#     flip_cam = False
#     detected = False
#     Controll = True
#     run = True
#     start_time_log = time.time()
#     start_time = time.time()
#     log_time = []
#     log_x = []
#     log_y = []
#     path = r"C:\Users\mateusz.jedynak\OneDrive - NTNU\Programmering\Python\Prosjekt\Bachelor\Source\Bachelor\Data\testing"

#     prev_angle_x = 0
#     prev_velocity_x = 0
#     while 1:
#         frames = pipeline.wait_for_frames()
#         aligned_frames =  align.process(frames)
#         depth_frame = aligned_frames.get_depth_frame()
#         aligned_color_frame = aligned_frames.get_color_frame()

        
#         image = np.asanyarray(aligned_color_frame.get_data())
#         depth = np.asanyarray(depth_frame.get_data())

#         camera_coordinates,detected = ObjectDetection(image,color_frame, depth_frame, lower_color, upper_color, flip_cam,True)
#         cv2.imshow("rsdfasd",image)
#         UR_10_pos = [0.03241,-0.88917,0.73515]
#         angle_x, angle_y = Angle(UR_10_pos[0],UR_10_pos[1],camera_coordinates[0],camera_coordinates[1],1.29)
#         # print(angle_x*180/math.pi,angle_y*180/math.pi)
#         # endtime = time.time()- start_time_log

#         #Angular acceleration
#         dt = time.time() - start_time
#         start_time = time.time()
#         x_velocity = (angle_x-prev_angle_x)/(dt)
#         prev_angle_x = angle_x

#         x_acceleration = (x_velocity-prev_velocity_x)/dt
#         prev_velocity_x = x_velocity

#         print(x_acceleration)

#         # log_time.append(endtime)
#         # log_x.append(angle_x)
#         # log_y.append(angle_y)
#         if cv2.waitKey(1) == 27:  # Break loop with ESC-key
#             # info_csv_1 = [f"Posisjonering til lasten er 62,5 grade fra UR10, Y: -140 X: -55"]
#             # info_csv_2 = ["  asdasd "]
#             # header = ["Time","X","Y"]
#             # with open(path + '\X-Y-ulike_PID_{}.csv'.format(str(len(os.listdir(path)))), 'w',newline="") as f:
#             #     # create the csv writer
#             #     writer = csv.writer(f)
#             #     writer.writerow(info_csv_1)
#             #     writer.writerow(info_csv_2)
#             #     writer.writerow(header)
#             #     for i in range(len(log_time)):
#             #         writer.writerow([log_time[i],log_x[i],log_y[i]])
#             print("Ferdig")   
#             break   
#     pipeline.stop()