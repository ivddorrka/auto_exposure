#!/usr/bin/env python

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
import dynamic_reconfigure.client
from std_msgs.msg import String
from threading import Thread, Lock

class TagDetectorExposureSimulation():
    
    def __init__(self, exposure):
        self.result = False
        self.exposure = exposure
        self.msg = None
        self.client = dynamic_reconfigure.client.Client("/camera/rgb_camera", timeout=30, config_callback=self.callback)
        self.client.update_configuration({"exposure":exposure})
        self.subscriber = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.get_tag)
        
    def callback(self, config):
        rospy.loginfo(f"Called back exposure = {self.exposure}")

    def get_tag(self, msg):
        if msg.detections != []:
            self.msg = msg
            rospy.set_param("detected_flag", 1)
        else:
            rospy.set_param("detected_flag", 0)
        self.subscriber.unregister()



def callback(config):
    rospy.loginfo("exposure callback")


if __name__ == "__main__":
    rospy.init_node("dynamic_exposure")
    i = 0
    r = rospy.Rate(1000)
    i_min = 0
    i_max = 10000
    rospy.set_param("detected_flag", 0)
    rospy.set_param("smallest_exposure", -1)
    rospy.set_param("largest_exposure", -1)
    pub = rospy.Publisher('results_top', String, queue_size=10)
    change_exposure_by = 100
    
    while not rospy.is_shutdown():
        
        cl = TagDetectorExposureSimulation(i)

        detected_flag = rospy.get_param("detected_flag")
        largest_expo = rospy.get_param("largest_exposure")
        smallest_expo = rospy.get_param("smallest_exposure")

        if detected_flag == 0:
            
            if largest_expo!=-1 and largest_expo>i and change_exposure_by>1 and i>100:
                i_max = i
                rospy.set_param("largest_exposure", i_max)
                change_exposure_by = int(change_exposure_by / 2)
                i -= change_exposure_by

            elif smallest_expo==-1 and largest_expo==-1:
                i +=1
            
            elif largest_expo!=-1 and smallest_expo!=-1:
                pub.publish(f"min exposure = {i_min}, max exposure = {i_max}, i_min = {i_min}, i_max = {i_max}")
                rospy.loginfo("Found min and max")
                rospy.signal_shutdown("The end")

            
        else:
            if i == 10000:
                i_max = i
                pub.publish(f"min exposure = {i_min}, max exposure = {i_max}, i_min = {i_min}, i_max = {i_max}")
                rospy.loginfo("Found min and max")
                rospy.signal_shutdown("The end")

            if smallest_expo==-1:
                i_min = i
                rospy.set_param("smallest_exposure", i)
                i += change_exposure_by

            elif smallest_expo!= -1 and change_exposure_by>1 and i<10000:
                i_max = i
                rospy.set_param("largest_exposure", i)
                
                i += change_exposure_by
                if i > 10000:
                    i = 10000

             


            #elif smallest_expo !=-1 and i<=i_max and largest_expo!=-1:

                #rospy.loginfo("minimum detecting exposure = {i_min}, maximum = {i_max}")
                #rospy.signal_shutdown("Found all")

        pub.publish(f"min exposure = {i_min}, max exposure = {i_max}, i_min = {i_min}, i_max = {i_max}")
        
        r.sleep()
