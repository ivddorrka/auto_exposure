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


class DynamicExposure:
    def __init__(self):
        
        self.client = dynamic_reconfigure.client.Client("/camera/rgb_camera", timeout=30, config_callback=self.exposure_change_callback)

        self.highest_exposure = 0
        self.lowest_exposure = 0
        self.current_exposure = 5000

        self.min_max_flag = 0

        self.lower_bound = 0
        self.upper_bound = 10000
        self.pub = rospy.Publisher('/exposure_boundaries', String, queue_size=10)       
        self.client.update_configuration({"exposure":self.current_exposure})
        self.search()

    def exposure_change_callback(self, msg):
        rospy.loginfo(f"Exposure set is {self.current_exposure}")

    def search(self):
        while True:
            rospy.sleep(1)

            data = rospy.wait_for_message("/tag_corners_in_px", String, timeout=30)
            rospy.sleep(1)

            data_str = str(data).split()[1]
            rospy.loginfo(data)
            found_tag_flag = 0
            if data_str!="''":
                found_tag_flag = 1

            rospy.loginfo(f"FOUND FLAG IS {found_tag_flag}")
            
            if (self.min_max_flag and found_tag_flag) or (not self.min_max_flag and not found_tag_flag):
                
                new_expo = int((self.current_exposure+self.upper_bound)/2)
                
                if new_expo==self.current_exposure:
                    if self.min_max_flag:
                        self.highest_exposure = self.current_exposure
                    else:
                        self.lowest_exposure = self.current_exposure
                    self.min_max_flag ^= 1

                    self.lower_bound = 0
                    self.current_exposure = 5000
                    self.upper_bound = 10000

                else:

                    self.lower_bound = self.current_exposure
                    self.current_exposure = new_expo
                   
            elif (self.min_max_flag and not found_tag_flag) or (not self.min_max_flag and found_tag_flag):
                
                new_expo = int((self.current_exposure+self.lower_bound)/2)
                
                if new_expo==self.current_exposure:
                    if self.min_max_flag:
                        self.highest_exposure = self.current_exposure
                    else:
                        self.lowest_exposure = self.current_exposure
                    self.min_max_flag ^= 1


                    self.lower_bound = 0
                    self.current_exposure = 5000
                    self.upper_bound = 10000
                
                else:
                    self.upper_bound = self.current_exposure
                    self.current_exposure = new_expo

            rospy.loginfo(f"EXPOSURE = {self.current_exposure}")
            self.pub.publish(String(f"Lowest exposure = {self.lowest_exposure}, Highest exposure = {self.highest_exposure}"))

            self.client.update_configuration({"exposure":self.current_exposure})

def main():
    rospy.init_node("dynamic_exposure")
    DynamicExposure()
    rospy.spin()


if __name__ == "__main__":
    main()


