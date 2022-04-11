import pyrealsense2 as rs
import numpy as np
import math
import cv2, keyboard
from cmath import sqrt
from Scripts.Camera_v2 import ObjectDetection,Inital_color

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)

align_to = rs.stream.depth
align = rs.align(align_to)
p1_x, p1_y = 773, 420
p2_x, p2_y = 600, 420
try:
    while True:
        # This call waits until a new coherent set of frames is available on a device
        frames = pipeline.wait_for_frames()
        
        #Aligning color frame to depth frame
        aligned_frames =  align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        aligned_color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not aligned_color_frame: continue

        color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(aligned_color_frame.get_data())
        #Use pixel value of  depth-aligned color image to get 3D axes

        cv2.circle(color_image, (p1_x,p1_y), 3, (0, 0, 255), -1)
        cv2.circle(color_image, (p2_x,p2_y), 3, (0, 255, 0), -1)
        cv2.imshow("Result", color_image)

        if keyboard.is_pressed("q"):
            p1_x +=1
        if keyboard.is_pressed("w"):
            p1_y +=1
        if keyboard.is_pressed("r"):
            p1_x -=1
        if keyboard.is_pressed("t"):
            p1_y -=1
        if keyboard.is_pressed("a"):
            p2_x +=1
        if keyboard.is_pressed("s"):
            p2_y +=1
        if keyboard.is_pressed("d"):
            p2_x -=1
        if keyboard.is_pressed("f"):
            p2_y -=1
        if cv2.waitKey(1) == 27:  # Break loop with ESC-key
            break   
        # b2 = sqrt(distance**2-distance2**2)
        # dx ,dy, dz = rs.rs2_deproject_pixel_to_point(color_intrin, [p1_x,p1_y], position_of_box)
        # dx2 ,dy2, dz2 = rs.rs2_deproject_pixel_to_point(color_intrin, [p2_x,p2_y], position_of_box)
        # distance2 = math.sqrt(((dx2)**2) + ((dy2)**2) + ((dz2)**2))
        # distance = math.sqrt(((dx)**2) + ((dy)**2) + ((dz)**2))





        position_of_box = depth_frame.get_distance(p1_x, p1_y)
        referance = depth_frame.get_distance(p2_x, p2_y)
        b = sqrt(position_of_box**2-referance**2) # den fungerer bedre 
        if b.imag == 0:
            b = b.real
        if b.real == 0:
            b = -b.imag
        # print(f"Distance from camera to pixel: {distance} Z-depth from camera surface to pixel surface: {depth}")
        print(b,p1_x,p2_x)

except Exception as e:
    print(e)
    pass

finally:
    pipeline.stop()