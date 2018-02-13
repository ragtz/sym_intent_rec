#!/usr/bin/env python

from cv_bridge import CvBridge, CvBridgeError
from sym_intent_rec.intent.color_filter import *
from sym_intent_rec.intent.blob_tracker import *
from sym_intent_rec.msg import WorldState
from sensor_msgs.msg import Image
import numpy as np
import rospy
import yaml
import sys

h_range = (150, 170)
s_range = (155, 255)
v_range = None

d_thresh = 30 # distance threshold

def get_cups(h_range, s_range, v_range, image=False):
    bridge = CvBridge()
    f_filter = color_filter(h_range, s_range, v_range, bridge)
    f_blobs = get_blobs(bridge, image=image)

    def f(msg):
        return f_blobs(f_filter(msg))

    return f

def check_present(i, p_check, p_curr):
    if len(p_curr) > 0:
        near = np.linalg.norm(np.array(p_check) - p_curr, axis=1) < d_thresh
        present = reduce(lambda x, y: x or y, near)
        return present
    else:
        return False

def publish_blobs(msg, args):
    cups_pos, pub = args
    pub.publish(cups_pos(msg))

def publish_state(msg, args):
    cups_pos, p_check, pub = args
    state = WorldState()

    p_curr = cups_pos(msg)

    for i, p in enumerate(p_check['right']):
        state.right.append(check_present(i, p, p_curr))
    
    for i, p in enumerate(p_check['left']):
        state.left.append(check_present(i, p, p_curr))

    pub.publish(state)

if __name__ == "__main__":
    p_check = yaml.load(open(sys.argv[1]))
    if len(sys.argv) > 2:
        view_blobs = bool(sys.argv[2])
    else:
        view_blobs = False

    rospy.init_node('extract_state')

    image_topic = '/kinect/qhd/image_color'
    state_topic = '/world_state'
    blobs_topic = '/cup_blobs'

    cups_pos = get_cups(h_range, s_range, v_range, view_blobs)

    if view_blobs:
        pub = rospy.Publisher(blobs_topic, Image, queue_size=10)
        rospy.Subscriber(image_topic, Image, publish_blobs, (cups_pos, pub))
    else:
        pub = rospy.Publisher(state_topic, WorldState, queue_size=10)
        rospy.Subscriber(image_topic, Image, publish_state, (cups_pos, p_check, pub))

    rospy.spin()

