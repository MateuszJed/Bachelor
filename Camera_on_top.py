"""
Code written based on servoj example from: https://github.com/davizinho5/RTDE_control_example
"""
import cv2,cmath,os,csv  # ,math,sys,asyncio,logging,time
import pyrealsense2 as rs
import numpy as np

#===============================functions===========================================================

#convert cameracoords to base coords.  #input: (camera coords, forward kin, joint 1 robot)
def Camera_top_to_qlobal_coords(camera_coords, translation, q1): 
    global_coords = np.array([[-np.sin(q1), np.cos(q1), 0], [np.cos(q1), np.sin(q1), 0], [0, 0, -1]]) @ np.c_[np.array(camera_coords)] + np.c_[translation]
    return (global_coords[0][0], global_coords[1][0], global_coords[2][0])



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
        if cv2.contourArea(box) > 1000:
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


def Inital_color(name_of_list):    # this function is not modified same as yours
    # """ Calibration center of body."""
    color_list = []
    with open(      #YOU NEED TO CHANGE TO YOUR FOLDER HERE!!!
            r"C:\Users\mateusz.jedynak\OneDrive - NTNU\Programmering\Python\Prosjekt\Bachelor\Source\Bachelor\Color_data_set\{}.txt".format(
                    name_of_list), "r") as f:
        for line in f:
            x = line.split(",")
            for i in x:
                if i.isdigit():
                    color_list.append(int(i))

    return np.array(color_list[:3]), np.array(color_list[3:])

#===============================End of functions=================================================================================
def Angle(UR_10_x,UR_10_y,object_x,object_y,l):
    return cmath.asin((object_x-UR_10_x)/l).real,cmath.asin((object_y-UR_10_y)/l).real
def Camera_top_to_qlobal_coords(camera_coords, translation, q1): 
    global_coords = np.array([[-np.sin(q1), np.cos(q1), 0], [np.cos(q1), np.sin(q1), 0], [0, 0, -1]]) @ np.c_[np.array(camera_coords)] + np.c_[translation]
    return (global_coords[0][0], global_coords[1][0], global_coords[2][0])

#the script used: 
cam_res = (848,480,60)
# cam_res = (1280,720,30)
lower_color, upper_color = Inital_color("redtop")
flip_cam = False
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, cam_res[0], cam_res[1], rs.format.bgr8, cam_res[2])
pipeline.start(config)

#CHANGE THERE VALUES EACH TIME YOU MOVE THE ROBOT: 
translation = [-19.4/ 1000, -931.84 / 1000, 712.92 / 1000] # forward kinematics.
angle = 91.01*cmath.pi/180                                              # angle of base
distance =  1389/ 1000                                       # distance to object/payload                                    # distance to object/payload
path = r"C:\Users\mateusz.jedynak\OneDrive - NTNU\Programmering\Python\Prosjekt\Bachelor\Source\Bachelor\Data\messurments"
x = []
y = []
def main():
    running = True
    log = False
    while 1:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        image = np.asanyarray(color_frame.get_data())
        coordinates_pix, coordinates_meters,detected = Object_3D_recontruction( image,          color_frame,
                                                                                lower_color,    upper_color, 
                                                                                flip_cam,       distance)
        global_coordinates = Camera_top_to_qlobal_coords(coordinates_meters, translation, angle) 

        cv2.putText(image, f"Global coordinates : {global_coordinates}", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255),2)

        cv2.imshow("Result", image)
        if log:
            coordinates_pix, coordinates_meters,detected = Object_3D_recontruction( image,          color_frame,
                                                                                lower_color,    upper_color, 
                                                                                flip_cam,       distance)
            global_coordinates = Camera_top_to_qlobal_coords(coordinates_meters, translation, angle) 
            x.append(global_coordinates[0])
            y.append(global_coordinates[1])
            if len(x) > 1000:
                info_csv_1 = [f"Posisjone til Lasten i globale kordinater "]
                #info_csv_2 = [
                    #f"P1:{}, Kp_y:{Kp_y}, Kd_x:{Kd_x}, Kd_y:{Kd_y}, Ki_x: {Ki_x}, Ki_x: {Ki_y}, referance point {reference_point_x}, {reference_point_y}"]
                header = ["X", "Y"]
                with open(path + '\Camptop_measur_cam_{}_{}_{}.csv'.format(cam_res[0],cam_res[1],str(len(os.listdir(path)))), 'w', newline="") as f:
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
