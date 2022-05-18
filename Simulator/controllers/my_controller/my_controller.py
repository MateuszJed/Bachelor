#!/home/kist-ubuntu/anaconda3/bin/python

""" Code author: DoranLyong 
    
    [Reference]:
    Go to the following links to check the specifications and parameters of the devices. 
        - Camera : https://cyberbotics.com/doc/reference/camera?tab-language=ros
        - Range_finder : https://cyberbotics.com/doc/guide/range-finder-sensors
        - UR10_e_Robots : https://cyberbotics.com/doc/guide/ure
"""
import sys 

import cv2

import rospy 
import rospkg
from std_msgs.msg import Float64
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import message_filters



from webots_ros.srv import (set_int, 
                            set_intRequest, 
                            get_int)
from rosgraph_msgs.msg import Clock


#%%
bridge = CvBridge()





#%%
# ================================================================= #
#                          Visual sensors                           #
# ================================================================= #




#%%
# ================================================================= #
#                            UR10 Robots                            #
# ================================================================= #


    

    






#%%
if __name__ =='__main__':



    """ Initialization 
    """
    rospy.init_node('UR10_Webots_World_Sensing_and_Control', anonymous=False)  # node initialize fist )


    """ RGB / Depth sensors 
    """   
  
    
    
    
    """ Object detection & classification 
    """



    """ UR10 controller publish
    """








    try:
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down...")
        
    cv2.destroyAllWindows()