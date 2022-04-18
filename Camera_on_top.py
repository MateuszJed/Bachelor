"""
Code written based on servoj example from: https://github.com/davizinho5/RTDE_control_example
"""
import cv2,cmath  # ,math,sys,asyncio,logging,time
import pyrealsense2 as rs
import numpy as np

#===============================functions===========================================================

#convert cameracoords to base coords.  #input: (camera coords, forward kin, joint 1 robot)
def Camera_top_to_qlobal_coords(camera_coords, translation, q1): 
    global_coords = np.array([[-np.sin(q1), np.cos(q1), 0], [np.cos(q1), np.sin(q1), 0], [0, 0, -1]]) @ np.c_[np.array(camera_coords)] + np.c_[translation]
    return (global_coords[0][0], global_coords[1][0], global_coords[2][0])



#convert pixel to meter. ( WITH ONLY USING THE RGB CAMERA) input distance is the distance to the object surface from the lens (center of camera)
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

            cv2.circle(image, [634, 358], 3, (0, 0, 255), -1)
            cv2.circle(image, center, 3, (0, 0, 255), -1)
            cv2.circle(image, (center[0] + 5, center[1] + 5), 3, (0, 0, 255), -1)
            cv2.circle(image, (center[0] - 5, center[1] - 5), 3, (0, 0, 255), -1)
            cv2.circle(image, (center[0] + 5, center[1] - 5), 3, (0, 0, 255), -1)
            cv2.circle(image, (center[0] - 5, center[1] + 5), 3, (0, 0, 255), -1)
            return center, (camera_x_meters, camera_y_meters, camera_z_meters)
        else:
            return (0, 0), (0, 0, 0)
    else:
        return (0, 0), (0, 0, 0)


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

#the script used: 

lower_color, upper_color = Inital_color("redcircle")

flip_cam = False
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)

#CHANGE THERE VALUES EACH TIME YOU MOVE THE ROBOT: 
translation = [-35.70/ 1000, -931.02 / 1000, 712.93 / 1000] # forward kinematics.
angle = 90                                                   # angle of base 
distance = 1300 / 1000                                       # distance to object/payload

def main():
    while 1:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        image = np.asanyarray(color_frame.get_data())

        # Find camera coords:
        coordinates_pix, coordinates_meters = Object_3D_recontruction(  image,          color_frame,
                                                                        lower_color,    upper_color, 
                                                                        flip_cam,       distance)
        # Camera coordinates:
        # print(f'X: {camera_x_meters*1000}    Y: {camera_y_meters*1000}    Z: {camera_z_meters*1000}')

        # Convert to Base/global coordinated:
        global_coordinates = Camera_top_to_qlobal_coords(coordinates_meters, translation, angle * np.pi / 180)

        # print camera coordinates:
        # print(f'Global coords:   X: {global_coordinates[0] * 1000}   Y: {global_coordinates[1] * 1000}    Z: {global_coordinates[2] * 1000}')

        # Diplay camera
        angle = Angle()
        cv2.putText(image, f"Angle_x : {reference_point_x}", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255),2)

        cv2.imshow("Result", image)
        if cv2.waitKey(1) == 27:  # (Break loop with ESC-key)
            pipeline.stop()
            break

if __name__ == '__main__':
    main()