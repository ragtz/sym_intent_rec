#!/usr/bin/env python

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import rospy
import cv2

params = cv2.SimpleBlobDetector_Params()

params.minThreshold = 250
params.maxThreshold = 255
params.thresholdStep = 10
params.minRepeatability = 1

params.filterByArea = True
params.minArea = 100
params.maxArea = 500000

params.filterByColor = False
params.filterByCircularity = False
params.filterByConvexity = False
params.filterByInertia = False

def get_blobs(bridge, image=True):
    detector = cv2.SimpleBlobDetector_create(params)

    def f(msg):
        img = bridge.imgmsg_to_cv2(msg, "mono8") 
        kp = detector.detect(img)
        
        if image:
            img = cv2.drawKeypoints(img, kp, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            return bridge.cv2_to_imgmsg(img, "bgr8")
        else:
            return [p.pt for p in kp]
    
    return f

if __name__ == "__main__":
    rospy.init_node('blob_tracker')

    in_topic = '/binary_image'
    out_topic = '/blobs'

    pub = rospy.Publisher(out_topic, Image, queue_size=10)    

    bridge = CvBridge()
    f = get_blobs(bridge)
    pub_blob_img = lambda msg: pub.publish(f(msg))

    rospy.Subscriber(in_topic, Image, pub_blob_img)

    rospy.spin()

