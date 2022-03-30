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
color_list = []
with open(r"D:\OneDrive - NTNU\Programmering\Python\Prosjekt\Bachelor\Source\Bachelor\Color_data_set\yellowbox.txt", "r") as f:
    for line in f:
        x = line.split(",")
    for i in x:
        if i.isdigit():
            color_list.append(int(i))

lower_color = np.array(color_list[:3])
upper_color = np.array(color_list[3:])

height = 720
width = 1280
while 1:
    frames = pipeline.wait_for_frames() #

    aligned_frames =  align.process(frames)
    depth_frame = aligned_frames.get_depth_frame()
    aligned_color_frame = aligned_frames.get_color_frame()

    image = np.asanyarray(aligned_color_frame.get_data())
    depth = np.asanyarray(depth_frame.get_data())

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Creates a mask
    mask = cv2.inRange(hsv, lower_color, upper_color)
    # Enlarge the mask
    kernel = np.ones((5, 5), np.uint8)
    dilation = cv2.dilate(mask, kernel)
    # Finding the contours
    contours, hierarchy = cv2.findContours(dilation, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        ball = max(contours, key=cv2.contourArea)
        if cv2.contourArea(ball) > 2000:
            cv2.drawContours(image, ball, -1, (0, 255, 0), 2)
            M = cv2.moments(ball)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            x_cord = center[0]-width/2
            y_cord = -1*(center[1]-height/2)

            cv2.circle(image, center, 3, (0, 0, 255), -1)
            cv2.putText(image, "centroid", (center[0] + 10, center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255),
                        1)
            cv2.putText(image, "(" + str(x_cord) + "," + str(y_cord) + ")", (center[0] + 10, center[1] + 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
            distance = depth_frame.get_distance(center[0], center[1])
            cv2.putText(image, str("Distance") + str(distance), (200,200),cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
    cv2.imshow("Result", image)
    cv2.imshow("Mask", mask)
    cv2.imshow("Depth", depth)
    if cv2.waitKey(1) == 27:  # Break loop with ESC-key
        break   