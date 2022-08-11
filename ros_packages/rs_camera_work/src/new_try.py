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
min_max_searching_flag = 0
lower_bound = 0
upper_bound = 10000
change_flag = 0
highest_exposure = 0
lowest_exposure = 0

def main():
    global highest_exposure, lowest_exposure
    rospy.init_node("rs_camera_exposure", anonymous=True)
    client = dynamic_reconfigure.client.Client("/camera/rgb_camera", timeout=30, config_callback=exposure_change_callback)
    client.update_configuration({"exposure":current_exposure})
    rospy.sleep(0.5)
    rospy.Subscriber("/tag_corners_in_px", String, ncallback, client)

    rospy.spin()


def exposure_change_callback(data):
    rospy.loginfo(f"{current_exposure}")

def ncallback(data, client):
    global current_exposure, min_max_searching_flag, change_flag
    global lower_bound, upper_bound
    global highest_exposure, lowest_exposure
    
    rospy.loginfo(f"New data = {data}") 

    
    if min_max_searching_flag:
    

        if len(str(data)) > 20:

            rospy.loginfo("Detected 1")
            rospy.sleep(2)
            new_midpoint = int((current_exposure+upper_bound)/2)
            lower_bound = current_exposure
            
            if new_midpoint == current_exposure:
                min_max_searching_flag = abs(min_max_searching_flag-1)
                current_exposure = 5000
                lower_bound = 0
                upper_bound = 10000

            else:
                current_exposure = new_midpoint
        
        else:
            rospy.loginfo("Not detected 1")
            rospy.sleep(2)
            new_midpoint = int((current_exposure+lower_bound)/2)
            upper_bound = current_exposure 
            
            if new_midpoint == current_exposure:
                min_max_searching_flag = abs(min_max_searching_flag-1)
                current_exposure = 5000
                lower_bound = 0
                upper_bound = 10000

            else:
                current_exposure = new_midpoint
       
    else:

        if len(str(data))>20:

            rospy.loginfo("Detected 2")
            rospy.sleep(2)
            new_midpoint = int((current_exposure+lower_bound)/2)
            upper_bound = current_exposure
             
            if new_midpoint == current_exposure:
                min_max_searching_flag = abs(min_max_searching_flag-1)
                current_exposure = 5000
                lower_bound = 0
                upper_bound = 10000
            else:
                current_exposure = new_midpoint
       
        else:
            
            rospy.loginfo("Not detected 2")
            rospy.sleep(2)
            new_midpoint = int((current_exposure+upper_bound)/2)
            lower_bound = current_exposure
              
            if new_midpoint == current_exposure:
                min_max_searching_flag = abs(min_max_searching_flag-1)
                current_exposure = 5000
                lower_bound = 0
                upper_bound = 10000
            else:
                current_exposure = new_midpoint
    client.update_configuration({"exposure":current_exposure})

def callback(data, client):
    
    global current_exposure #the exposure at which the tag is either detected or not with the current data in callback

    global min_max_searching_flag, change_flag #flags concerning which exposure to find, second flag here is to stop searching both of them in an infinite loop, when they've been already found
    global lower_bound, upper_bound # bounds to make binary search possible

    global highest_exposure, lowest_exposure #these two variables are being set on each iteration, while rospy is NOT shutdown, they provide info about the lowest and highest possible exposure found so far
    
    rospy.loginfo(f"{data}")
    rospy.loginfo(f"{lower_bound} <= {current_exposure} <= {upper_bound}")
    #rospy.sleep(2)

    if (len(str(data)) > 20 and min_max_searching_flag==1) or (min_max_searching_flag==0 and len(str(data))<20): # combined these into one "IF" because for both these situations the following changes will be applied

        rospy.loginfo(f"Difference 1  = {-(current_exposure - upper_bound)}")   

        if (upper_bound-current_exposure)>1: #checking if there's any sense to be searching for either lowest or heighest exposure further

            if min_max_searching_flag:
            
                highest_exposure = current_exposure # setting current highest exposure found
            
            else:
                
                lowest_exposure = current_exposure # setting current lowest exposure found

            new_n = int((current_exposure+upper_bound)/2) #this is the new midpoint of this binary search - it will be in the right half from the previous midpoint 
            lower_bound = current_exposure
            current_exposure = new_n

        else:   #this executes if the difference is 1 or less -> which means that either lowest or highest exposure is already found

            if change_flag!=0: #this is the counter of how much the min_max_searching_flag has been changed (it shoulf only be changed once

                rospy.loginfo(f"Minimum exposure = {lowest_exposure}, maximum exposure = {highest_exposure}")
                rospy.signal_shutdown("Found both")


            min_max_searching_flag = abs(min_max_searching_flag-1)
            current_exposure = 5000
            lower_bound = 0
            upper_bound = 10000
            change_flag +=1
            # the parameters from line 71-75 will be changed in the corresponding way, so that the other exposure bound could be found

    elif (min_max_searching_flag==1 and len(str(data))<20) or (min_max_searching_flag==0 and len(str(data))>20):
        #in the following part of the code happens the oppposite from the previous one
        # regarding the above stated "IF" the search will continue in the left part from the corresponding midpoint (current midpoint is the current_exposure in the search

        rospy.loginfo(f"Difference 2 = {current_exposure-lower_bound}")   
        
        if (current_exposure-lower_bound)>1:

            if (min_max_searching_flag):

                highest_exposure = current_exposure

            else:

                lowest_exposure = current_exposure
            
            new_n = int((current_exposure+lower_bound)/2)
            upper_bound = current_exposure
            current_exposure = new_n

        else:

            if change_flag!=0:

                rospy.loginfo(f"Minimum exposure = {lowest_exposure}, maximum exposure = {highest_exposure}")
                rospy.signal_shutdown("Found both")

            min_max_searching_flag = abs(min_max_searching_flag-1)
            current_exposure = 5000
            lower_bound = 0
            upper_bound = 10000   
            change_flag += 1
            # the parameters from line 96-100 will be changed in the corresponding way, so that the other exposure bound could be found


    client.update_configuration({"exposure":current_exposure}) #after all of the above changes - the current exposure will have new value - in each itearaion the current exposure is the midpoint of a binary search, at which APtag either is being detected or not
    


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass





















