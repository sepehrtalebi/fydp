import rospy
from bmb_msgs.msg import OpticalFlowReading
from sensor_msgs.msg import Image

import cv2
import numpy as np
import matplotlib.pyplot as plt
from time import perf_counter

#global vars
prvs = 0

class OpticalFlow:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/rectified_image", Image, self.image_callback)
        self.optical_flow_pub = rospy.Publisher("/optical_flow_reading", OpticalFlowReading, queue_size=1)

    def image_callback(self, image):
        
        #converting ROS message to opencv image
        frame2 = bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
        next = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY)
        
        if prvs!=0:          
            flow = cv2.calcOpticalFlowFarneback(prvs, next, None,0.5, 3, 15, 3, 5, 1.2, 0)
            if 1==2: #np.var(flow) > ? ---- still determining best way to do variance formula. Will update. 
                velocity = None
            else:    
                velocity = np.mean(flow, axis = (0,1))
            prvs = next
        else:
            velocity = (0,0)
            
        optical_flow_reading = velocity

        self.optical_flow_pub.publish(optical_flow_reading)
        
        

    def spin(self):
        rospy.spin()
