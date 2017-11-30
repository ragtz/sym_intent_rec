#!/usr/bin/env python

from cv_bridge import CvBridge, CvBridgeError
from sym_intent_rec.intent.color_filter import *
from sym_intent_rec.intent.blob_tracker import *
from sensor_msgs.msg import Image
from std_msgs.msg import String
from collections import deque
from copy import deepcopy
import numpy as np
import rospy

n = 1 # number of consecutive detections to classify as left/right
d_thresh = 15 # min magnitude of motion to classify as left/right
a_min = np.pi/3 # min angle from y-axis to classify as left/right
a_max = 3*np.pi/4 # max angle from y-axis to classify as left/right

AND = lambda l: reduce(lambda x, y: x and y, l)
HAS_NONE = lambda l: reduce(lambda x, y: x or y, map(lambda x: x is None, l))

def get_hand(h_range, s_range, v_range):
    bridge = CvBridge()
    f_filter = color_filter(h_range, s_range, v_range, bridge)
    f_blobs = get_blobs(bridge, image=False)
    
    def f(msg):
        return f_blobs(f_filter(msg))

    return f

# returns 'l', 'r', '-'
def detect_motion(p, p_last):
    m_type = '-'
    # was hand detected
    if p and not HAS_NONE(p_last):
        p_arr = np.array(p[0])
        motion = p_arr - p_last
        mag = np.linalg.norm(motion)
        ang = np.arccos(np.dot([0,1], motion)/mag)

        # was motion large enough
        if mag > d_thresh:
            # was not moving up or down
            if ang > a_min and ang < a_max:
                if motion[0] > 0:
                    m_type = 'l'
                else:
                    m_type = 'r'

    if p:
        p = p[0]
        p_last[0] = p[0]
        p_last[1] = p[1]

    return m_type

def publish_intent(msg, args):
    hand_pos, last_pos, motion_history, pub = args

    h = hand_pos(msg)
    motion = detect_motion(h, last_pos) 
             
    motion_history.append(motion)
    if AND(np.array(motion_history) == 'l'):
        intent = 'left'
    elif AND(np.array(motion_history) == 'r'):
        intent = 'right'
    else:
        intent = 'none'

    pub.publish(intent)

if __name__ == "__main__":
    rospy.init_node('intent_recognition')

    image_topic = '/kinect2/qhd/image_color'
    intent_topic = '/intent'

    hand_pos = get_hand(h_range, s_range, v_range)
    last_pos = [None, None]
    motion_history = deque([], n)

    pub = rospy.Publisher(intent_topic, String, queue_size=10)
    rospy.Subscriber(image_topic, Image, publish_intent, (hand_pos, last_pos, motion_history, pub))

    rospy.spin()

