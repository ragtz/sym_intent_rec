#!/usr/bin/env python

from sensor_msgs.msg import Image
from sym_intent_rec.world.extract_state import *
import rospy
import yaml
import sys

def get_left_right(positions, mid=0):
    left = filter(lambda p: p[0] < mid, positions)
    right = filter(lambda p: p[0] >= mid, positions)
    return left, right 

def save(msg, args):
    cups_pos, filename = args

    positions = cups_pos(msg)
    left, right = get_left_right(positions, msg.width/2.)

    data = {'left': left,
            'right': right}

    yaml.dump(data, open(filename, 'w'))

    print "n:", len(positions)
    print "left:", len(left)
    print "right:", len(right)
    
if __name__ == "__main__":
    filename = sys.argv[1]

    rospy.init_node('save_state')
    image_topic = '/kinect/qhd/image_color'

    cups_pos = get_cups(h_range, s_range, v_range)

    msg = rospy.wait_for_message(image_topic, Image, timeout=5)
    save(msg, (cups_pos, filename))

