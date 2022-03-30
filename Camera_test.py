import pyrealsense2 as rs
import numpy as np
import math,cv2

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)

align_to = rs.stream.depth
align = rs.align(align_to)

try:
    while True:
        # This call waits until a new coherent set of frames is available on a device
        frames = pipeline.wait_for_frames()
        
        #Aligning color frame to depth frame
        color_frame = frames.get_color_frame()
        aligned_frames =  align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        image = np.asanyarray(color_frame.get_data())
        if not depth_frame or not aligned_frames: continue

        depth_image = np.asanyarray(depth_frame.get_data())

        #Use pixel value of  depth-aligned color image to get 3D axes
        x, y = 640, 360
        depth = depth_frame.get_distance(x, y)
        cv2.circle(image, (x,y), 3, (0, 0, 255), -1)
        print("Z-depth from camera surface to pixel surface:", depth)
        cv2.imshow("Result", depth_image)
        cv2.imshow("Mask", image)
        if cv2.waitKey(1) == 27:  # Break loop with ESC-key
            break   
finally:
    pipeline.stop()