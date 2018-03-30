#!/usr/bin/env python

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import rospy
import cv2

h_range = (80, 120)
s_range = (140, 255)
#s_range = (150, 255)
#s_range = (180, 255)
v_range = None

def process_range(r):
    if r is None:
        return (0, 255)
    else:
        return r

def color_filter(h_range, s_range, v_range, bridge):
    h_range = process_range(h_range)
    s_range = process_range(s_range)
    v_range = process_range(v_range)

    def f(msg):
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower = np.array([h_range[0], s_range[0], v_range[0]])
        upper = np.array([h_range[1], s_range[1], v_range[1]])
        
        img = cv2.inRange(img, lower, upper)
        #img = cv2.bitwise_and(img, img, mask=mask)
        msg = bridge.cv2_to_imgmsg(img, "mono8")

        return msg

    return f

if __name__ == "__main__":
    rospy.init_node('color_filter')

    pre_filter_topic = '/kinect/qhd/image_color'
    post_filter_topic = '/binary_image'

    pub = rospy.Publisher(post_filter_topic, Image, queue_size=10)

    bridge = CvBridge()
    f = color_filter(h_range, s_range, v_range, bridge)
    pub_filt_img = lambda msg: pub.publish(f(msg))

    rospy.Subscriber(pre_filter_topic, Image, pub_filt_img)

    rospy.spin()

