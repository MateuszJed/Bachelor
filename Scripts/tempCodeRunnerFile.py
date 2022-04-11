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
        camera_coordinates,center, detected = ObjectDetection(image,color_frame, depth_frame, lower_color, upper_color, flip_cam)
        x,y,z = Camera_to_global_coords(camera_coordinates[0],camera_coordinates[1],camera_coordinates[2])
        
        print(x*1000,y*1000,z*1000,center)
    pipeline.stop()