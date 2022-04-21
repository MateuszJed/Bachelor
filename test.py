## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#####################################################
##           librealsense T265 example             ##
#####################################################

# First import the library
import pyrealsense2 as rs

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)

while 1:
    # Wait for the next set of frames from the camera
    frames = pipeline.wait_for_frames()

    # Fetch pose frame
    pose = frames.get_pose_frame()
    print(pose)
    if pose:
        # Print some of the pose data to the terminal
        data = pose.get_pose_data()
        print("Frame #{}".format(pose.frame_number))
        print("Position: {}".format(data.translation))
        print("Velocity: {}".format(data.velocity))
        print("Acceleration: {}\n".format(data.acceleration))

