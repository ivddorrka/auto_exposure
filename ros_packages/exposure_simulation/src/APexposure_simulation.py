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

# if 0 -> minimum exposure will be found
# otherwise max will be found
global flag_min_or_max

# callback for colo image subscriber
def ImageCallback(image_message):
    rospy.loginfo("Made an rgb image callback")

    
# called when exposure changes
def ReconfigureCallback(imsg):
    rospy.loginfo("New exposure set")

def tagDetector_callback(msg):
    #if msg.detections!=[]:
    rospy.loginfo(msg)
    #rospy.signal_shutdown(msg)


#main logic in tagdetect
def TagDetectedCallback(tag_detections_message, args):
    
    flag_min_or_max = args

    publisher = rospy.Publisher('exposure_info', String, queue_size=10)

    client = dynamic_reconfigure.client.Client("/camera/rgb_camera", timeout=30, config_callback=ReconfigureCallback)


    current_exposure = 0
    if flag_min_or_max==0:
        rospy.loginfo(f"current exposure = {current_exposure}")
        rospy.loginfo(f"TAG DETECTIONS = {tag_detections_message.detections}")
        client.update_configuration({"exposure":current_exposure})
        rospy.sleep(1.5)

        if tag_detections_message.detections != []:
            publisher.publish(f"the seeking exposure = {current_exposure}")
            rospy.signal_shutdown("Found needed exposure")
        else:
            rospy.loginfo(tag_detections_message)
            current_exposure += 1
            publisher.publish(f"current exposure = {current_exposure}")


# the function to find the smallest or the largest exposure
# depending on the min_or_max_flag 
# if it's 0 - then the smallest will, otherwise the largest
def main_so_far(setter_to_a_flag):
    flag_min_or_max = setter_to_a_flag
    rospy.init_node("dynamic_exposure")
    current_exposure = 0
    
    client = dynamic_reconfigure.client.Client("/camera/rgb_camera", timeout = 30, config_callback=ReconfigureCallback)
    rospy.sleep(1)

    while not rospy.is_shutdown():
        client.update_configuration({"exposure":current_exposure})
        #image_subscriber = rospy.Subscriber("camera/realsense2_camera/color/image_raw", Image, ImageCallback)
        #rospy.sleep(1.5) #to give time to subscribe and execute ImageCallback function
    
        tag_detections_subscriber = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, tagDetector_callback)
        rospy.sleep(1)
        rospy.loginfo("subscribed")
        current_exposure += 1
    #rospy.spin()   


if __name__ == "__main__":
    setter_to_a_flag = 0 # if 0 - then the minimum exposure will be found
    # if that is set to 1 -> then the maximum exposure will be found
    main_so_far(0)

