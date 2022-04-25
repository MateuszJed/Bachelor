import cv2,cmath
import numpy as np
import pyrealsense2 as rs
def Camera_to_global_coords(x,y,z):
    global_coords= np.array(np.array([[1,0,0,0],[0,0,1,-2.184],[0,-1,0,-0.662],[0,0,0,1]]))@np.array([[x],[y],[z],[1]])
    return global_coords[0][0],global_coords[1][0],global_coords[2][0]
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

def ObjectDetection(image,color_frame, depth_frame, lower_color, upper_color, flip_cam):
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
            distance = np.mean(distance[distance != 0])+0.07

            intrin = color_frame.profile.as_video_stream_profile().intrinsics
            camera_coordinates_3D= np.array([rs.rs2_deproject_pixel_to_point(intrin, [center[0], center[1]],
                                                                              distance),
                                              rs.rs2_deproject_pixel_to_point(intrin, [center[0] + 5, center[1] + 5],
                                                                              distance),
                                              rs.rs2_deproject_pixel_to_point(intrin, [center[0] - 5, center[1] - 5],
                                                                              distance),
                                              rs.rs2_deproject_pixel_to_point(intrin, [center[0] + 5, center[1] - 5],
                                                                              distance),
                                              rs.rs2_deproject_pixel_to_point(intrin, [center[0] - 5, center[1] + 5],
                                                                              distance)])


            camera_x_meters = np.mean(camera_coordinates_3D[:,0])
            camera_y_meters = np.mean(camera_coordinates_3D[:, 1])
            camera_z_meters = np.mean(camera_coordinates_3D[:, 2][camera_coordinates_3D[:, 2] != 0])

            cv2.circle(image, center, 3, (0, 0, 255), -1)
            cv2.putText(image, "centroid", (center[0] + 10, center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255),1)

            
            x,y,z = Camera_to_global_coords(camera_x_meters,camera_y_meters,camera_z_meters)
            
            cv2.circle(image, center, 3, (0, 0, 255), -1)
            cv2.putText(image, "centroid", (center[0] + 10, center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255),1)
            # cv2.putText(image, "(" + str(x_cord) + ")", (center[0] + 10, center[1] + 15),
            #             cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            # cv2.imshow("Result", image)

            return [x,y,z],True
        else:
            return [0,0,0],False
    else:
            return [0,0,0],False


if __name__ == "__main__":
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    pipeline.start(config)
    align_to = rs.stream.depth
    align = rs.align(align_to)

    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    image = np.asanyarray(color_frame.get_data())


    lower_color, upper_color = Inital_color("yellowbox")
    flip_cam = False
    detected = False
    Controll = True
    run = True
    
    while 1:
        frames = pipeline.wait_for_frames()
        aligned_frames =  align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        aligned_color_frame = aligned_frames.get_color_frame()

        
        image = np.asanyarray(aligned_color_frame.get_data())
        depth = np.asanyarray(depth_frame.get_data())

        # cv2.imshow("Reeeeesult", image)
        camera_coordinates, detected = ObjectDetection(image,color_frame, depth_frame, lower_color, upper_color, flip_cam)
        x = camera_coordinates[0]
        y = camera_coordinates[1]
        z = camera_coordinates[2]
        print(x*1000,y*1000,z*1000)
    pipeline.stop()