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


found_flag = [0, 1]

def exposure_change_callback(msg):
    rospy.loginfo(f"New exposure set = {msg.exposure}") 

def tag_detect_callback(tg_msg, found_flag):
    found_flag[0] = 0
    rospy.loginfo(f"TAG MESSAGE is {tg_msg}")
    if len(str(tg_msg)) > 20:
        found_flag[0] = 1
        found_flag[1] = tg_msg

def image_callback(msg, args):
    args[1][0] = args[0].imgmsg_to_cv2(msg)

def AP_contrast_im(img):
    im_size = [len(img), len(img[0])]
    contrast = (np.amax(img) - np.amin(img))/(np.sum(img)/im_size[0]/im_size[1])
    return contrast




def binary_search(find_min_or_max_flag, client):
    upper_bound = 10000 # the max exposure to start detecting AP with
    lower_bound = 0 # the min exposure to start detecting AP with
    current_exposure = 5000 # the mid to start searching with 
    continue_searching_flag = 1 # if 0 it means that either max or min exposure has been found
    br = CvBridge() #cv bridge to convert ros images

    while not rospy.is_shutdown() and continue_searching_flag:
 
        client.update_configuration({"exposure":current_exposure})
        rospy.sleep(0.5)
        
        current_image = [0] #after image callback the first element of this list will be the opencv image
        current_image_subscriber = rospy.Subscriber("/camera/realsense2_camera/color/image_raw", Image, image_callback, [br, current_image])
        rospy.sleep(0.5)

        tag_detect_subscriber = rospy.Subscriber("/tag_corners_in_px", String, tag_detect_callback, found_flag)
        rospy.sleep(0.5)

        if found_flag[0]:
            
            increase_part = abs(int(0.5*(upper_bound-current_exposure)))
            if increase_part>1:
                if find_min_or_max_flag:
                    lower_bound = current_exposure
                    current_exposure += increase_part 
                else:
                    upper_bound = current_exposure
                    current_exposure -= increase_part
            else:
                continue_searching_flag=0
            rospy.loginfo(str(found_flag[0]))
            crop_distance = re.split("\[(.*?)\]", str(found_flag[1]))

            result_list = [crop_distance[i].split(", ") for i in range(len(crop_distance)) if i%2!=0]
            rospy.loginfo(result_list)
            lst = []
            for i in range(len(result_list)):
                lst_prom = []
                for j in range(0, 2):
                    lst_prom.append(int(result_list[i][j]))
                lst.append(lst_prom)

            rospy.loginfo(f"Cropped info is = {lst}")
            
            x_middle = math.floor(abs(lst[0][0]+lst[2][0])/2)
            y_middle = math.floor(abs(lst[0][1]+lst[2][1])/2)


            im_cropped = current_image[0][y_middle-100:y_middle+100, x_middle-100:x_middle+100]

            cntr = AP_contrast_im(im_cropped)
            rospy.loginfo(cntr)
            
            #plt.imshow(im_cropped)
            #plt.show()
            #rospy.signal_shutdown("the end")
        
        else:
            decrease_part = abs(int(0.5*(current_exposure-lower_bound)))
            if decrease_part >1: 
                if find_min_or_max_flag:
                    upper_bound = current_exposure
                    current_exposure -= decrease_part
                else:
                    lower_bound = current_exposure
                    current_exposure += decrease_part
            else:
                continue_searching_flag = 0
    
    if not continue_searching_flag:
        return current_exposure


if __name__ == "__main__":
    rospy.init_node("dynamic_exposure")
    client = dynamic_reconfigure.client.Client("/camera/rgb_camera", timeout=30, config_callback=exposure_change_callback)
    
    min_exposure = binary_search(0, client)
    max_exposure = binary_search(1, client) #if first argument is zero - then the lowes possible exposure will be found, otherwise the largest

    rospy.loginfo(f"Lowest exposure is = {min_exposure}, largest exposure = {max_exposure}")
    rospy.signal_shutdown("Found min and max exposure!")



