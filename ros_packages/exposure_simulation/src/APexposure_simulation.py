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


def exposure_change_callback(msg):
    rospy.loginfo(f"New exposure set, {msg}")

def tag_detect_callback(tg_msg, found_flag):
    found_flag[0] = 0
    if tg_msg.detections:
        found_flag[0] = 1
    rospy.loginfo(tg_msg)

def binary_search(find_min_or_max_flag, client):
    upper_bound = 10000 # the max exposure to start detecting AP with
    lower_bound = 0 # the min exposure to start detecting AP with
    current_exposure = 5000 # the mid to start searching with 
    continue_searching_flag = 1
    while not rospy.is_shutdown() and continue_searching_flag:
        client.update_configuration({"exposure":current_exposure})
        rospy.sleep(2)

        found_flag = [0]
        tag_detect_subscriber = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, tag_detect_callback, found_flag)
        rospy.sleep(2)
        
        if find_min_or_max_flag:
            if found_flag[0]:
                lower_bound = current_exposure
                increase_part = int(0.5*(upper_bound-current_exposure))
                if increase_part>3:
                    current_exposure += increase_part 
                else:
                    continue_searching_flag=0
            else:
                upper_bound = current_exposure
                decrease_part = int(0.5*(current_exposure-lower_bound))
                if decrease_part >3: 
                    current_exposure -= decrease_part
                else:
                    continue_searching_flag = 0
        else:
            if found_flag[0]:
                lower_bound = current_exposure
                append_part = int(0.5*(upper_bound-current_exposure))
                if append_part>3:
                    current_exposure -= int(0.5*(upper_bound - current_exposure))
                else:
                    continue_searching_flag=0
            else:
                upper_bound = current_exposure
                decrease_part = int(0.5*(current_exposure-lower_bound))
                if decrease_part >3: 
                    current_exposure += int(0.5*(current_exposure-lower_bound))
                else:
                    continue_searching_flag = 0
           



        


if __name__ == "__main__":
    rospy.init_node("dynamic_exposure")
    client = dynamic_reconfigure.client.Client("/camera/rgb_camera", timeout=30, config_callback=exposure_change_callback)
    
    binary_search(1, client) #if first argument is zero - then the lowes possible exposure will be found, otherwise the largest



