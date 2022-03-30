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
    x, y = 640, 230
    depth = depth_frame.get_distance(x, y)
    dx ,dy, dz = rs.rs2_deproject_pixel_to_point(color_intrin, [x,y], depth)
    distance = math.sqrt(((dx)**2) + ((dy)**2) + ((dz)**2))
    cv2.circle(color_image, (x,y), 3, (0, 0, 255), -1)
    print(f"Distance toPix: {distance}, surface {depth}")
    cv2.imshow("Result", depth_image)
    cv2.imshow("Mask", color_image)
    if cv2.waitKey(1) == 27:  # Break loop with ESC-key
        break