import cv2
import numpy as np

def Inital_color(name_of_list):
    # """ Calibration center of body."""
    color_list = []
    with open(r"D:\OneDrive - NTNU\Programmering\Python\Prosjekt\Bachelor\Source\Color_data_set\{}.txt".format(name_of_list), "r") as f:
        for line in f:
            x = line.split(",")
            for i in x:
                if i.isdigit():
                    color_list.append(int(i))

    return np.array(color_list[:3]), np.array(color_list[3:])

def ObjectDetection(image,lower_color, upper_color,height,width,flip_cam):
    # Convert RGB to HSV
    if flip_cam:
        image = cv2.flip(image,-1)
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
            return x_cord, y_cord,mask,image,True
        else:
            return 0, 0,mask,image, False
    else:
            return 0, 0,mask,image, False


