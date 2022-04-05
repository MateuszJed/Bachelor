#.\Prosjekt\Bachelor_object_detection\Aruco-env\Scripts\activate
#cd .\Prosjekt\Bachelor_object_detection\Color_detection\
import pyrealsense2 as rs
import numpy as np
import cv2
import torch
from matplotlib import pyplot as plt
# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)

# cap = cv2.VideoCapture(0)
def colorCalibration():
    """Defining color and save as list to the path.
    Define path to list for save """
    nameOfList = input("What is name for your list: ")
    def nothing(x):
      pass
    # Creates a window containing trackbars
    Trackbars = np.zeros([100, 700], np.uint8)
    cv2.namedWindow("Trackbars")
    cv2.createTrackbar("L - H", "Trackbars", 0, 179, nothing)
    cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("L - V", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("U - H", "Trackbars", 179, 179, nothing)
    cv2.createTrackbar("U - S", "Trackbars", 255, 255, nothing)
    cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)
    cv2.createTrackbar('S ROWS', 'Trackbars', 0, 480, nothing)
    cv2.createTrackbar('E ROWS', 'Trackbars', 480, 480, nothing)
    cv2.createTrackbar('S COL', 'Trackbars', 0, 640, nothing)
    cv2.createTrackbar('E COL', 'Trackbars', 640, 640, nothing)


    while True:
        # Take each frame

        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        # Convert images to numpy arrays
        image = np.asanyarray(color_frame.get_data())
        # succes, image = cap.read()
        	
        image = cv2.flip(image, 0)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        s_r = cv2.getTrackbarPos('S ROWS', 'Trackbars')
        e_r = cv2.getTrackbarPos('E ROWS', 'Trackbars')
        s_c = cv2.getTrackbarPos('S COL', 'Trackbars')
        e_c = cv2.getTrackbarPos('E COL', 'Trackbars')

        # Reads trackbar position
        l_h = cv2.getTrackbarPos("L - H", "Trackbars")
        l_s = cv2.getTrackbarPos("L - S", "Trackbars")
        l_v = cv2.getTrackbarPos("L - V", "Trackbars")
        u_h = cv2.getTrackbarPos("U - H", "Trackbars")
        u_s = cv2.getTrackbarPos("U - S", "Trackbars")
        u_v = cv2.getTrackbarPos("U - V", "Trackbars")
        # Assign trackbar values to HSV-limits.
        lower_blue = np.array([l_h, l_s, l_v])
        upper_blue = np.array([u_h, u_s, u_v])

        # Creates a mask
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Show frame different frames
        cv2.imshow("Result", image)
        cv2.imshow("Mask", mask)
        colorlist = [l_h,l_s,l_v,u_h,u_s,u_v]
    
        if cv2.waitKey(1) == 27:  # Break loop with ESC-key
            with open(r"C:\Users\mateusz.jedynak\OneDrive - NTNU\Programmering\Python\Prosjekt\Bachelor\Source\Bachelor\Color_data_set\{}.txt".format(nameOfList), "w") as f:
              for s in colorlist:
                f.write(str(s) +",")
            break

def videoCalibration():
    """ Calibration center of body."""
    color_list = []
    name_of_list = input("Name of list: ")
    with open(r"C:\Users\mateusz.jedynak\OneDrive - NTNU\Programmering\Python\Prosjekt\Bachelor\Source\Bachelor\Color_data_set\{}.txt".format(name_of_list), "r") as f:
      for line in f:
        x = line.split(",")
        for i in x:
            if i.isdigit():
                color_list.append(int(i))

    lower_color = np.array(color_list[:3])
    upper_color = np.array(color_list[3:])
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        # Convert images to numpy arrays
        image = np.asanyarray(color_frame.get_data())
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # Convert RGB to HSV
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
          if cv2.contourArea(box) > 1800:
            ball = max(contours, key=cv2.contourArea)     
            area = cv2.contourArea(ball) 
            print(area)
            cv2.drawContours(image, ball, -1, (0, 255, 0), 2)
            M = cv2.moments(ball)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            cv2.circle(image, center, 3, (0, 0, 255), -1)
            cv2.putText(image, "centroid", (center[0] + 10, center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255),
                        1)
            cv2.putText(image, "(" + str(center[0]) + "," + str(center[1]) + ")", (center[0] + 10, center[1] + 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
        cv2.imshow("Result", image)
        cv2.imshow("Mask", mask)
        if cv2.waitKey(1) == 27:  # Break loop with ESC-key
            break   