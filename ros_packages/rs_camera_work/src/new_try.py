#!/usr/bin/env python

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
import dynamic_reconfigure.client
from std_msgs.msg import String
from threading import Thread, Lock
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import math
import cv2 as cv
import matplotlib.pyplot as plt
import re
from PIL import Image as pil_image

current_exposure = 5000
min_max_searching_flag = 1
lower_bound = 0
upper_bound = 10000
change_flag = 0
highest_exposure = 0
lowest_exposure = 0

def main():
    rospy.init_node("nnode", anonymous=True)
    client = dynamic_reconfigure.client.Client("/camera/rgb_camera", timeout=30, config_callback=exposure_change_callback)
    client.update_configuration({"exposure":current_exposure})
    rospy.Subscriber("/tag_corners_in_px", String, callback, client)
    rospy.spin()



def exposure_change_callback(data):
    rospy.loginfo(f"{current_exposure}")

def callback(data, client):
    global current_exposure
    global min_max_searching_flag
    global lower_bound
    global upper_bound
    global change_flag
    global highest_exposure
    global lowest_exposure

    rospy.loginfo(data)
    
    if (len(str(data)) > 20 and min_max_searching_flag==1) or (min_max_searching_flag==0 and len(str(data))<20):
        rospy.loginfo(f"Difference = {upper_bound-current_exposure}")
        if (upper_bound-current_exposure)>1:
            new_n = int((current_exposure+upper_bound)/2)
            lower_bound = current_exposure
            current_exposure = new_n
        else:  
            if min_max_searching_flag:
                highest_exposure = current_exposure
            else:
                lowest_exposure = current_exposure

            if change_flag!=0:

                rospy.loginfo(f"Minimum exposure = {lowest_exposure}, maximum exposure = {highest_exposure}")
                rospy.signal_shutdown("Found both")


            min_max_searching_flag = abs(min_max_searching_flag-1)
            current_exposure = 5000
            lower_bound = 0
            upper_bound = 10000
            change_flag +=1

        rospy.loginfo(f"{min_max_searching_flag} is a flag")

    elif (min_max_searching_flag==1 and len(str(data))<20) or (min_max_searching_flag==0 and len(str(data))>20):
        rospy.loginfo(f"Difference = {current_exposure-lower_bound}")   
        if (current_exposure-lower_bound)>1:
            new_n = int((current_exposure+lower_bound)/2)
            upper_bound = current_exposure
            current_exposure = new_n
        else:
            if min_max_searching_flag:
                highest_exposure = current_exposure
            else:
                lowest_exposure = current_exposure

            if change_flag!=0:

                rospy.loginfo(f"Minimum exposure = {lowest_exposure}, maximum exposure = {highest_exposure}")
                rospy.signal_shutdown("Found both")

            min_max_searching_flag = abs(min_max_searching_flag-1)
            current_exposure = 5000
            lower_bound = 0
            upper_bound = 10000   
            change_flag += 1

        rospy.loginfo(f"{min_max_searching_flag} is a flag")

    client.update_configuration({"exposure":current_exposure})
    


def TagCallback(data, client):
    global current_exposure
    global min_max_searching_flag
    global lower_bound
    global upper_bound
    rospy.loginfo(f"{data}, CURRENT EXPOSURE = {current_exposure}")

    if (min_max_searching_flag==1 and len(str(data))>20) or (min_max_searching_flag==0 and len(str(data))<20):
        rospy.loginfo(f"mmflag = {min_max_searching_flag}, lenstrdata = {len(str(data))}")

        new_middle_exposure = int((current_exposure+upper_bound)/2)
        lower_bound = current_exposure
        current_exposure = new_middle_exposure
        rospy.loginfo(f"new are - {lower_bound} <= {current_exposure} <= {upper_bound}")    

    if (min_max_searching_flag==0 and len(str(data))>20) or (len(str(data))<20 and min_max_searching_flag==1):
        rospy.loginfo(f"mmflag = {min_max_searching_flag}, lenstrdata = {len(str(data))}")
        new_middle_exposure = int((lower_bound+current_exposure)/2)
        upper_bound = current_exposure
        current_exposure = new_middle_exposure
        rospy.loginfo(f"new are - {lower_bound} <= {current_exposure} <= {upper_bound}")
   
    client.update_configuration({"exposure":current_exposure})
    rospy.sleep(3)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass





















